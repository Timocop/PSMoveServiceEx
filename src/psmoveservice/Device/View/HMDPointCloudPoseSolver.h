#ifndef HMD_POINT_CLOUD_POSE_SOLVER_H
#define HMD_POINT_CLOUD_POSE_SOLVER_H

#include <opencv2/core.hpp>

#include <utility>
#include <vector>

struct HMDPointCloudPosePrior
{
    HMDPointCloudPosePrior()
        : orientation_rvec(0., 0., 0.)
        , position_cm(0., 0., 0.)
        , position_age_seconds(0.)
        , has_orientation(false)
        , has_position(false)
    {
    }

    // Both values transform model-space points into camera space.
    cv::Vec3d orientation_rvec;
    cv::Vec3d position_cm;
    double position_age_seconds;
    bool has_orientation;
    bool has_position;
};

struct HMDPointCloudPoseSolverSettings
{
    HMDPointCloudPoseSolverSettings()
        : min_inlier_count(5)
        , min_continuation_inlier_count(4)
        , max_association_distance_px(60.)
        , min_association_distance_px(12.)
        , max_reprojection_error_px(8.)
        , min_strict_gate_px(8.)
        , max_strict_gate_px(22.)
        , min_depth_cm(30.)
        , max_depth_cm(300.)
        , max_lateral_position_cm(200.)
        , min_model_pair_distance_cm(4.5)
        , min_ray_angle_degrees(0.5)
        , min_blob_pair_distance_px(4.)
        , max_pair_translation_residual_cm(4.)
        , coarse_geometry_tolerance_cm(5.)
        , strict_geometry_tolerance_cm(2.)
        , max_normalized_rms_error(0.55)
        , min_projection_area_px_sqr(25.)
        , min_acquisition_coverage(0.20)
        , min_continuation_coverage(0.12)
        , max_orientation_error_degrees(45.)
        , max_acquisition_tilt_error_degrees(15.)
        , max_acquisition_heading_error_degrees(35.)
        , max_continuation_tilt_error_degrees(8.)
        , max_continuation_heading_error_degrees(18.)
        , max_translation_jump_cm(45.)
        , base_translation_gate_cm(8.)
        , translation_gate_speed_cm_per_second(120.)
        , max_position_prior_age_seconds(0.25)
        , acquisition_yaw_step_degrees(12.)
        , acquisition_ambiguity_score_margin(75.)
        , continuation_ambiguity_score_margin(25.)
    {
    }

    int min_inlier_count;
    int min_continuation_inlier_count;

    // Coarse assignment gate:
    // clamp(4px + focal_length * coarse_geometry_tolerance / depth,
    //       min_association_distance_px, max_association_distance_px).
    double max_association_distance_px;
    double min_association_distance_px;

    // Final strict assignment and acceptance gates.
    double max_reprojection_error_px;
    double min_strict_gate_px;
    double max_strict_gate_px;

    double min_depth_cm;
    double max_depth_cm;
    double max_lateral_position_cm;

    double min_model_pair_distance_cm;
    double min_ray_angle_degrees;
    double min_blob_pair_distance_px;
    double max_pair_translation_residual_cm;
    double coarse_geometry_tolerance_cm;
    double strict_geometry_tolerance_cm;

    double max_normalized_rms_error;
    double min_projection_area_px_sqr;
    double min_acquisition_coverage;
    double min_continuation_coverage;

    double max_orientation_error_degrees;
    double max_acquisition_tilt_error_degrees;
    double max_acquisition_heading_error_degrees;
    double max_continuation_tilt_error_degrees;
    double max_continuation_heading_error_degrees;

    double max_translation_jump_cm;
    double base_translation_gate_cm;
    double translation_gate_speed_cm_per_second;
    double max_position_prior_age_seconds;

    // Acquisition tests {-2, -1, 0, +1, +2} times this yaw step.
    double acquisition_yaw_step_degrees;

    double acquisition_ambiguity_score_margin;
    double continuation_ambiguity_score_margin;
};

struct HMDPointCloudPoseSolverDiagnostics
{
    HMDPointCloudPoseSolverDiagnostics()
        : orientation_hypothesis_count(0)
        , pair_hypothesis_count(0)
        , accepted_pair_hypothesis_count(0)
        , coarse_hypothesis_count(0)
        , retained_coarse_candidate_count(0)
        , exact_assignment_count(0)
        , unique_mapping_count(0)
        , pnp_attempt_count(0)
        , successful_pnp_count(0)
        , retained_pose_candidate_count(0)
    {
    }

    int orientation_hypothesis_count;
    int pair_hypothesis_count;
    int accepted_pair_hypothesis_count;
    int coarse_hypothesis_count;
    int retained_coarse_candidate_count;
    int exact_assignment_count;
    int unique_mapping_count;
    int pnp_attempt_count;
    int successful_pnp_count;
    int retained_pose_candidate_count;
};

struct HMDPointCloudPoseResult
{
    HMDPointCloudPoseResult()
        : orientation_rvec(0., 0., 0.)
        , position_cm(0., 0., 0.)
        , reprojection_error_px(0.)
        , projection_area_px_sqr(0.)
        , projection_coverage(0.)
        , score(0.)
        , ambiguous(false)
    {
    }

    cv::Vec3d orientation_rvec;
    cv::Vec3d position_cm;
    double reprojection_error_px;
    double projection_area_px_sqr;
    double projection_coverage;
    double score;
    bool ambiguous;

    // Pairs are (model point index, original image point index).
    std::vector<std::pair<int, int> > inlier_pairs;
    HMDPointCloudPoseSolverDiagnostics diagnostics;
};

class HMDPointCloudPoseSolver
{
public:
    enum
    {
        kMaxModelPointCount = 7,
        kMaxImagePointCount = 16,
        kMaxOrientationHypothesisCount = 5,
        kMaxRawPairHypothesisCount = 25200,
        kMaxCoarseCandidateCount = 128,
        kMaxMappingHypothesisCount = 3,
        kMaxPoseCandidateCount = 8,
        kMaxPnPAttemptCount = 82
    };

    static bool solve(
        const std::vector<cv::Point3f> &model_points_cm,
        const std::vector<cv::Point2f> &image_points_px,
        const cv::Matx33d &camera_matrix,
        const cv::Vec<double, 5> &distortion_coefficients,
        const HMDPointCloudPosePrior &prior,
        const HMDPointCloudPoseSolverSettings &settings,
        HMDPointCloudPoseResult &result);
};

#endif // HMD_POINT_CLOUD_POSE_SOLVER_H
