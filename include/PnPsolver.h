/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#ifndef PNPSOLVER_H
#define PNPSOLVER_H

#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "Frame.h"

namespace ORB_SLAM2
{

class PnPsolver {
 public:
   /**构造函数
    * @param vpMapPointMatches 保存F中的特征点与与哪些mappoint匹配，vpMapPointMatches[i]表示F中第i个特征点所指向的mappoint
    */
  PnPsolver(const Frame &F, const vector<MapPoint*> &vpMapPointMatches);

  ~PnPsolver();

  //设置参数
  void SetRansacParameters(double probability = 0.99, int minInliers = 8 , int maxIterations = 300, int minSet = 4, float epsilon = 0.4,
                           float th2 = 5.991);

  cv::Mat find(vector<bool> &vbInliers, int &nInliers);

  //循环nIterations次执行epnp算法
  cv::Mat iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers);

 private:

   //对于此次RANSAC计算的epnp求得的位姿，原先F中的特征点与mappoint的匹配还有哪些成立
   //更新mnInliersi
  void CheckInliers();
  //以mvbBestInliers中的点对通过epnp计算位姿而不是先前使用4个点对计算位姿
  //如果计算的结果对应的inliner超过阈值mRansacMinInliers，则返回成功
  bool Refine();

  // Functions from the original EPnP code
  void set_maximum_number_of_correspondences(const int n);
  void reset_correspondences(void);
  // 将对应的3D-2D压入到pws和us
  void add_correspondence(const double X, const double Y, const double Z,
              const double u, const double v);

  //通过epnp计算相机位姿
  double compute_pose(double R[3][3], double T[3]);

  void relative_error(double & rot_err, double & transl_err,
              const double Rtrue[3][3], const double ttrue[3],
              const double Rest[3][3],  const double test[3]);

  void print_pose(const double R[3][3], const double t[3]);
  double reprojection_error(const double R[3][3], const double t[3]);

  //获得EPnP算法中的四个控制点
  void choose_control_points(void);
  void compute_barycentric_coordinates(void);
  void fill_M(CvMat * M, const int row, const double * alphas, const double u, const double v);
  void compute_ccs(const double * betas, const double * ut);
  void compute_pcs(void);

  void solve_for_sign(void);

  void find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho, double * betas);
  void qr_solve(CvMat * A, CvMat * b, CvMat * X);

  double dot(const double * v1, const double * v2);
  double dist2(const double * p1, const double * p2);

  void compute_rho(double * rho);
  void compute_L_6x10(const double * ut, double * l_6x10);

  void gauss_newton(const CvMat * L_6x10, const CvMat * Rho, double current_betas[4]);
  void compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
				    double cb[4], CvMat * A, CvMat * b);

  double compute_R_and_t(const double * ut, const double * betas,
			 double R[3][3], double t[3]);

  void estimate_R_and_t(double R[3][3], double t[3]);

  void copy_R_and_t(const double R_dst[3][3], const double t_dst[3],
		    double R_src[3][3], double t_src[3]);

  void mat_to_quat(const double R[3][3], double q[4]);


  //相机内参
  double uc, vc, fu, fv;
  //pws,us为RANSAC生成的3d，2d坐标点的容器
  //alphas为pws，pcs在控制点下的hd坐标
  //pws，pcs分别为3d点的世界坐标和相机坐标
  double * pws, * us, * alphas, * pcs;
  int maximum_number_of_correspondences;
  int number_of_correspondences;

  //4个控制点在世界坐标系和相机坐标系下的坐标
  double cws[4][3], ccs[4][3];
  double cws_determinant;

  //mvpMapPointMatches[i]表示F中第i个特征点所指向的mappoint
  vector<MapPoint*> mvpMapPointMatches;

  // 2D Points
  //F的mappoint对应的F的特征点的像素坐标
  vector<cv::Point2f> mvP2D;
  //与高斯金字塔有关,F的特征点数量大小
  vector<float> mvSigma2;

  // 3D Points
  //F的mappoint的3d世界坐标，F的特征点数量大小
  vector<cv::Point3f> mvP3Dw;

  // Index in Frame
  //mvP2D,mvP3Dw对应F的特征点在F中的序号
  vector<size_t> mvKeyPointIndices;

  // Current Estimation
  //当前估计的位姿
  double mRi[3][3];
  double mti[3];
  cv::Mat mTcwi;
  //对于每次RANSAC计算出的位姿，哪些点对属于inliners
  vector<bool> mvbInliersi;
  //mvbInliersi中属于true的有多少
  int mnInliersi;

  // Current Ransac State
  int mnIterations;
  //mnBestInliers对应那次RANSAC的mvbInliersi
  vector<bool> mvbBestInliers;
  //已执行的RANSAC中，最大的mnInliersi值
  int mnBestInliers;
  //mnBestInliers对应的那次RANSAC所得的位姿
  cv::Mat mBestTcw;

  // Refined
  cv::Mat mRefinedTcw;
  //Refine()中算出的mnInliersi
  vector<bool> mvbRefinedInliers;
  int mnRefinedInliers;

  // Number of Correspondences
  //匹配点对的数量
  int N;

  // Indices for random selection [0 .. N-1]
  //[0,..,N]的序列，N为F的特征点数量大小
  vector<size_t> mvAllIndices;

  // RANSAC probability
  double mRansacProb;

  // RANSAC min inliers
  //一次RANSAC迭代成功的阈值
  int mRansacMinInliers;

  // RANSAC max iterations
  //RANSAC最大累计迭代次数阈值
  int mRansacMaxIts;

  // RANSAC expected inliers/total ratio
  float mRansacEpsilon;

  // RANSAC Threshold inlier/outlier. Max error e = dist(P1,T_12*P2)^2
  float mRansacTh;

  // RANSAC Minimun Set used at each iteration
  //mRansacMinSet为每次RANSAC需要的特征点数
  int mRansacMinSet;

  // Max square error associated with scale level. Max error = th*th*sigma(level)*sigma(level)
  vector<float> mvMaxError;

};

} //namespace ORB_SLAM

#endif //PNPSOLVER_H
