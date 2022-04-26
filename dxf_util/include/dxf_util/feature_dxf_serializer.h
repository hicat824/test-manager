//
// Created by xumaozhou on 2022/4/21.
//
#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <nlohmann/json.hpp>

class DL_Dxf;
class DL_WriterA;

namespace kd {
namespace mapping {
namespace dxf_util {

class FeatureDxfSerializer {
  using PtVec = std::vector<Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> >;
  using DLWriterAPtr = std::shared_ptr<DL_WriterA>;
  using DLDxfPtr = std::shared_ptr<DL_Dxf>;
public:
  FeatureDxfSerializer() {
  };
  virtual ~FeatureDxfSerializer();

  /**
   * 写入DXF文件标头等基础信息
   * @param filename
   */
  void DxfWriterInit(const std::string &filename);

  /**
   * flush data && close file
   */
  void DxfWriterClose();

  /**
   * 点数据写入DXF文件
   * @param data
   * @param coord_sys : 1--unchange,2 : wgs->utm ,3 : wgs->mgrs,4 : wgs
   * @return
   */
  int AddPoint(const Eigen::Vector3d &data, const int coord_sys);

  /**
   * 线数据写入DXF文件
   * @param data
   * @param coord_sys : 1--wgs,2 : wgs->utm ,3 : wgs->mgrs,4 : unchange
   * @return
   */
  int AddLine(const PtVec &data, const int coord_sys);

  /**
   * geojson转dxf
   * @param file_in
   * @param datatype : 1--Point,2--LineString,3--Polygon,4--all
   * @param coord_sys : 1--wgs,2 : wgs->utm ,3 : wgs->mgrs,4 : unchange
   * @param file_out
   * @return
   */
  int GeoJsonReader(const std::string &file_in,  const int datatype, const int coord_sys, const std::string &file_out);

protected:
  /**
   * BLH转换至UTM坐标系
   * @param wgs
   * @param utm
   */
  void BLHToUTM(const Eigen::Vector3d &lla, Eigen::Vector3d &utm);

  /**
   * wgs转mgrs坐标系(返回对应网格原点坐标以及对应key）
   * @param wgs
   * @param prec : 0 is 100km, 6 is 0.1m
   * @param mgrs : 网格原点坐标
   */
  std::string BLHToMGRS(const Eigen::Vector3d &lla, int prec, Eigen::Vector3d &mgrs);

  /**
   * 坐标变换
   * @param pt
   * @param coord_sys : 1--wgs,2 : wgs->utm ,3 : wgs->mgrs,4 : unchange
   * @return ptconvert
   */
  Eigen::Vector3d CoordinateTransform(const Eigen::Vector3d &pt, const int coord_sys);

private:
  DLDxfPtr dxf_;
  DLWriterAPtr dw_;
};
}//namespace dxf_util
}//namespace mapping
}//namespace kd