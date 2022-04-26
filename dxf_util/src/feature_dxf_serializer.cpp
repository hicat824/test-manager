//
// Created by xumaozhou on 2022/4/21.
//

#include "feature_dxf_serializer.h"

#include <gflags/gflags.h>
#include <glog/logging.h>

#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/MGRS.hpp>

#include "dxflib/dl_dxf.h"
//#include "lidarpoint_util/coordinates/mgrs_utils.h"

//using namespace kd::mapping::lidarpoint_util;
using namespace kd::mapping::dxf_util;
using namespace GeographicLib;
using json = nlohmann::json;

FeatureDxfSerializer::~FeatureDxfSerializer() {
  DxfWriterClose();
}

void FeatureDxfSerializer::BLHToUTM(const Eigen::Vector3d &lla, Eigen::Vector3d &utm) {
  int zone;
  bool northp;
  UTMUPS::Forward(lla[1],lla[0],zone,northp,utm[0],utm[1], GeographicLib::UTMUPS::STANDARD, true);
  utm[2] = lla[2];
}

std::string FeatureDxfSerializer::BLHToMGRS(const Eigen::Vector3d &lla, int prec, Eigen::Vector3d &mgrs) {
  int zone;
  bool northp;
  UTMUPS::Forward(lla[1],lla[0],zone,northp,mgrs[0],mgrs[1],GeographicLib::UTMUPS::STANDARD, true);
  std::string key;
  MGRS::Forward(zone, northp,mgrs[0],mgrs[1],lla[1], prec,key);
  //MGRS::Reverse(mg,zone,northp,mgrs[0],mgrs[1],prec,false);
  mgrs[2] = lla[2];
  return key;
}

Eigen::Vector3d FeatureDxfSerializer::CoordinateTransform(const Eigen::Vector3d &pt, const int coord_sys) {
  Eigen::Vector3d ptconvert;
  switch (coord_sys) {
    case 1://unchange
      ptconvert = pt;
      break;
    case 2://utm
      BLHToUTM(pt,ptconvert);
      break;
    case 3://mgrs
      BLHToMGRS(pt,5,ptconvert);
      break;
    case 4://wgs
      ptconvert = pt;
      break;
    default:
      LOG(FATAL) << "unkown coord system " << coord_sys;
      break;
  }
  return ptconvert;
}

void FeatureDxfSerializer::DxfWriterInit(const std::string &filename) {
  //init
  dxf_ = std::make_shared<DL_Dxf>();
  dw_.reset(new DL_WriterA(filename.c_str(), DL_Codes::AC1015));

  if(dw_->openFailed()) {
    LOG(FATAL) << "initialization failed";
    return;
  }
  //section Header
  dxf_->writeHeader(*dw_);
  dw_->sectionEnd();
  // section tables:
  dw_->sectionTables();
  // VPORT:
  dxf_->writeVPort(*dw_);
  // LTYPE:常用线类型声明
  dw_->tableLinetypes(3);
  dxf_->writeLinetype(*dw_, DL_LinetypeData("CONTINUOUS", "Continuous", 0, 0, 0.0));
  dxf_->writeLinetype(*dw_, DL_LinetypeData("BYLAYER", "", 0, 0, 0.0));
  dxf_->writeLinetype(*dw_, DL_LinetypeData("BYBLOCK", "", 0, 0, 0.0));
  dw_->tableEnd();
  // LAYER:图层设定
  dw_->tableLayers(2);
  dxf_->writeLayer(
      *dw_,
      DL_LayerData("layer-0", 0),
      DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS"));
  dxf_->writeLayer(*dw_,
                 DL_LayerData("layer-1", 0),
                 DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS"));
  dxf_->writeLayer(*dw_,
                 DL_LayerData("layer-2", 0),
                 DL_Attributes("", 1, 0x00ff0000, 15, "CONTINUOUS"));
  dw_->tableEnd();
  // DIMSTYLE:
  dxf_->writeDimStyle(*dw_, 2.5, 0.625, 0.625, 0.625, 2.5);
  // BLOCK_RECORD:
  dxf_->writeBlockRecord(*dw_);
  dw_->tableEnd();

  dw_->sectionEnd();//end tables

  // BLOCK:始终包含三个空定义Model_Space、Paper_Space与Paper_Space0
  dw_->sectionBlocks();
  dxf_->writeBlock(*dw_, DL_BlockData("*Model_Space", 0, 0.0, 0.0, 0.0));
  dxf_->writeEndBlock(*dw_, "*Model_Space");
  dxf_->writeBlock(*dw_, DL_BlockData("*Paper_Space", 0, 0.0, 0.0, 0.0));
  dxf_->writeEndBlock(*dw_, "*Paper_Space");
  dxf_->writeBlock(*dw_, DL_BlockData("*Paper_Space0", 0, 0.0, 0.0, 0.0));
  dxf_->writeEndBlock(*dw_, "*Paper_Space0");
  dw_->sectionEnd();//end BLOCK

  //ENTITIES--图形需要放在同一个实体段才能在qgis中加载出来
  dw_->sectionEntities();
}

void FeatureDxfSerializer::DxfWriterClose() {
  if(dw_ != nullptr) {
    //FLUSH
    dw_->FlushStream();
    dw_->sectionEnd();// end section ENTITIES
    //write object
    dxf_->writeObjects(*dw_, "MY_OBJECTS");
    dxf_->writeObjectsEnd(*dw_);
    //close file
    dw_->dxfEOF();
    dw_->close();
    dw_ = nullptr;
  } else {
    LOG(FATAL) << "Please Init First !";
  }
}


int FeatureDxfSerializer::AddPoint(const Eigen::Vector3d &data, const int coord_sys) {
  if (dxf_ == nullptr || dw_ == nullptr) {
    LOG(FATAL) << "Please Init First !";
    return -1;
  }
  Eigen::Vector3d p1;
  p1 = CoordinateTransform(data, coord_sys);
  dxf_->writePoint(*dw_,
                  DL_PointData(p1[0], p1[1], p1[2]),
                  DL_Attributes("layer-0", 256, -1, "BYLAYER", 1.0));
  return 0;
}

int FeatureDxfSerializer::AddLine(const PtVec &data, const int coord_sys) {
  if (dxf_ == nullptr || dw_ == nullptr) {
    LOG(FATAL) << "Please Init First !";
    return -1;
  }
  if (data.size() < 2) {
    return -2;
  }

  Eigen::Vector3d p1, p2;
  for (size_t i = 1 ; i < data.size() ; i++) {
    p1 = CoordinateTransform(data[i-1], coord_sys);
    p2 = CoordinateTransform(data[i], coord_sys);;
    dxf_->writeLine(*dw_,
                  DL_LineData(p1[0], p1[1], p1[2],   // start point
                              p2[0], p2[1], p2[2]),   // end point
                  DL_Attributes("layer-1", 256, -1, "BYLAYER", 1.0));
  }
  return 0;
}


int FeatureDxfSerializer::GeoJsonReader(const std::string &file_in,  const int datatype, const int coord_sys, const std::string &file_out) {
  if(dxf_ == nullptr || dw_ == nullptr) {
    DxfWriterInit(file_out);
  }

  std::ifstream file_(file_in);
  if (!file_.is_open()) {
    LOG(FATAL) << "file " << file_in << " does not exist !";
    return -3;
  }
  int res = 0;
  json nj = json::parse(file_);
  for (auto &it : nj["features"]) {
    if (it.contains("geometry")) {
      auto objects = it["geometry"];
      std::string  data_type;
      objects.at("type").get_to(data_type);
      if(data_type == "Point" && (datatype == 1 || datatype ==4)) {
        Eigen::Vector3d  pt;
        for(int i=0; i<3; i++) {
          pt[i]=objects["coordinates"].at(i).get<double_t>();
        }
        res = AddPoint(pt, coord_sys);
      }
      if(data_type == "LineString" && (datatype == 2 || datatype ==4)) {
        PtVec line_data;
        for (auto &pt : objects["coordinates"]) {
          line_data.push_back(Eigen::Vector3d(pt[0], pt[1], pt[2]));
        }
        res = AddLine(line_data, coord_sys);
      }
      if(data_type == "Polygon" && (datatype == 3 || datatype ==4)) {
        PtVec polygon_vertex;
        auto points = objects["coordinates"].at(0);
        for (auto &pt : points) {
          polygon_vertex.push_back(Eigen::Vector3d(pt[0], pt[1], pt[2]));
        }
        res = AddLine(polygon_vertex, coord_sys);
      }
    }
  }
  return res;
}
