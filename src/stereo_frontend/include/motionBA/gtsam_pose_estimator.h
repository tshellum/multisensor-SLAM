#ifndef LAB_7_VO_GTSAM_POSE_ESTIMATOR_H
#define LAB_7_VO_GTSAM_POSE_ESTIMATOR_H

#include "pose_estimator.h"
#include "gtsam/geometry/Cal3_S2.h"

/// \brief Iterative pose estimator for calibrated camera with 3D-2D correspondences.
/// This pose estimator need another pose estimator,
/// which it will use to initialize estimate and find inliers.
class GtsamPoseEstimator : public PoseEstimator
{
public:
  /// \brief Constructs pose estimator.
  /// \param initial_pose_estimator Pointer to a pose estimator for initialization and inlier extraction.
  /// \param K Camera calibration matrix.
  GtsamPoseEstimator(PoseEstimator::Ptr initial_pose_estimator,
                     const Eigen::Matrix3d& K);

  /// \brief Estimates camera pose from 3D-2D correspondences.
  /// \param image_points 2D image points.
  /// \param world_points 3D planar world points.
  /// \return The results. Check PoseEstimate::isFound() to check if solution was found.
  PoseEstimate estimate(const std::vector<cv::Point2f>& image_points,
                        const std::vector<cv::Point3f>& world_points) override;

private:
  PoseEstimator::Ptr initial_pose_estimator_;
  Eigen::Matrix3d K_;
  gtsam::Cal3_S2::shared_ptr K_gtsam_;
};

#endif //LAB_7_VO_GTSAM_POSE_ESTIMATOR_H
