#include "HMDPointCloudPoseSolver.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <tuple>
#include <vector>

namespace
{
    const double kInvalidAssignmentCost = 1000000.;
    const double kUnmatchedAssignmentCost = 10.;
    const double kMinimumPointDepthCm = 1.;
    const double kTranslationQuantizationCm = 1.;
    const double kPoseClusterTranslationCm = 2.5;
    const double kPoseClusterRotationDegrees = 2.5;

    struct ObservedPoint
    {
        cv::Point2f pixel;
        cv::Point2d normalized;
        cv::Vec3d ray;
        int original_index;
    };

    struct OrientationHypothesis
    {
        cv::Matx33d rotation;
        cv::Vec3d rvec;
        double absolute_yaw_degrees;
        int index;
    };

    struct TranslationCandidate
    {
        TranslationCandidate()
            : orientation_index(0)
            , position_cm(0., 0., 0.)
            , pair_residual_cm(0.)
            , prior_distance_cm(0.)
            , absolute_yaw_degrees(0.)
            , coarse_match_count(0)
            , coarse_normalized_squared_error(0.)
            , model_index_a(-1)
            , model_index_b(-1)
            , observed_index_a(-1)
            , observed_index_b(-1)
        {
        }

        int orientation_index;
        cv::Vec3d position_cm;
        double pair_residual_cm;
        double prior_distance_cm;
        double absolute_yaw_degrees;
        int coarse_match_count;
        double coarse_normalized_squared_error;
        int model_index_a;
        int model_index_b;
        int observed_index_a;
        int observed_index_b;
    };

    struct AssignmentResult
    {
        AssignmentResult()
            : match_count(0)
            , normalized_squared_error(0.)
        {
        }

        int match_count;
        double normalized_squared_error;
        std::vector<std::pair<int, int> > pairs;
    };

    struct MappingHypothesis
    {
        TranslationCandidate candidate;
        AssignmentResult assignment;
    };

    struct PoseCandidate
    {
        PoseCandidate()
            : orientation_rvec(0., 0., 0.)
            , position_cm(0., 0., 0.)
            , match_count(0)
            , reprojection_error_px(std::numeric_limits<double>::max())
            , maximum_normalized_error(std::numeric_limits<double>::max())
            , normalized_rms_error(std::numeric_limits<double>::max())
            , projection_area_px_sqr(0.)
            , projection_coverage(0.)
            , total_orientation_error_degrees(std::numeric_limits<double>::max())
            , tilt_error_degrees(std::numeric_limits<double>::max())
            , heading_error_degrees(std::numeric_limits<double>::max())
            , translation_error_cm(0.)
            , score(-std::numeric_limits<double>::max())
        {
        }

        cv::Vec3d orientation_rvec;
        cv::Vec3d position_cm;
        std::vector<std::pair<int, int> > assignments;
        int match_count;
        double reprojection_error_px;
        double maximum_normalized_error;
        double normalized_rms_error;
        double projection_area_px_sqr;
        double projection_coverage;
        double total_orientation_error_degrees;
        double tilt_error_degrees;
        double heading_error_degrees;
        double translation_error_cm;
        double score;
    };

    static double clamp_double(const double value, const double minimum, const double maximum)
    {
        return std::max(minimum, std::min(maximum, value));
    }

    static bool is_finite(const cv::Vec3d &value)
    {
        return
            std::isfinite(value[0]) &&
            std::isfinite(value[1]) &&
            std::isfinite(value[2]);
    }

    static bool is_finite(const cv::Point2f &value)
    {
        return std::isfinite(value.x) && std::isfinite(value.y);
    }

    static bool is_finite(const cv::Point3f &value)
    {
        return
            std::isfinite(value.x) &&
            std::isfinite(value.y) &&
            std::isfinite(value.z);
    }

    static cv::Vec3d to_vec3d(const cv::Point3f &point)
    {
        return cv::Vec3d(
            static_cast<double>(point.x),
            static_cast<double>(point.y),
            static_cast<double>(point.z));
    }

    static bool rotation_from_rvec(
        const cv::Vec3d &rvec,
        cv::Matx33d &rotation)
    {
        if (!is_finite(rvec))
        {
            return false;
        }

        cv::Mat rotation_mat;
        try
        {
            cv::Rodrigues(rvec, rotation_mat);
        }
        catch (const cv::Exception &)
        {
            return false;
        }

        if (rotation_mat.rows != 3 ||
            rotation_mat.cols != 3 ||
            rotation_mat.type() != CV_64F)
        {
            return false;
        }

        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                rotation(row, column) = rotation_mat.at<double>(row, column);
            }
        }

        const double determinant = cv::determinant(rotation);
        return std::isfinite(determinant) && determinant > 0.99 && determinant < 1.01;
    }

    static bool rvec_from_rotation(
        const cv::Matx33d &rotation,
        cv::Vec3d &rvec)
    {
        cv::Mat rvec_mat;
        try
        {
            cv::Rodrigues(rotation, rvec_mat);
        }
        catch (const cv::Exception &)
        {
            return false;
        }

        if (rvec_mat.total() != 3 || rvec_mat.type() != CV_64F)
        {
            return false;
        }

        rvec = cv::Vec3d(
            rvec_mat.at<double>(0),
            rvec_mat.at<double>(1),
            rvec_mat.at<double>(2));
        return is_finite(rvec);
    }

    static double angle_between_degrees(
        const cv::Vec3d &left,
        const cv::Vec3d &right)
    {
        const double left_length = cv::norm(left);
        const double right_length = cv::norm(right);
        if (left_length <= 1e-12 || right_length <= 1e-12)
        {
            return std::numeric_limits<double>::max();
        }

        const double cosine =
            clamp_double(left.dot(right) / (left_length * right_length), -1., 1.);
        return std::acos(cosine) * (180. / CV_PI);
    }

    static double rotation_error_degrees(
        const cv::Matx33d &left,
        const cv::Matx33d &right)
    {
        const cv::Matx33d delta = left * right.t();
        const double trace =
            delta(0, 0) + delta(1, 1) + delta(2, 2);
        const double cosine =
            clamp_double((trace - 1.) * 0.5, -1., 1.);
        return std::acos(cosine) * (180. / CV_PI);
    }

    static void compute_tilt_and_heading_errors(
        const cv::Matx33d &orientation,
        const cv::Matx33d &prior_orientation,
        double &tilt_error_degrees,
        double &heading_error_degrees)
    {
        const cv::Vec3d model_up(0., 1., 0.);
        const cv::Vec3d model_forward(0., 0., 1.);
        const cv::Vec3d prior_up = prior_orientation * model_up;
        const cv::Vec3d orientation_up = orientation * model_up;

        tilt_error_degrees =
            angle_between_degrees(prior_up, orientation_up);

        const double prior_up_length = cv::norm(prior_up);
        if (prior_up_length <= 1e-12)
        {
            heading_error_degrees = std::numeric_limits<double>::max();
            return;
        }

        const cv::Vec3d up_axis = prior_up * (1. / prior_up_length);
        cv::Vec3d prior_forward = prior_orientation * model_forward;
        cv::Vec3d orientation_forward = orientation * model_forward;
        prior_forward -= up_axis * prior_forward.dot(up_axis);
        orientation_forward -= up_axis * orientation_forward.dot(up_axis);

        heading_error_degrees =
            angle_between_degrees(prior_forward, orientation_forward);
    }

    static bool observed_point_less(
        const ObservedPoint &left,
        const ObservedPoint &right)
    {
        if (left.pixel.x != right.pixel.x)
        {
            return left.pixel.x < right.pixel.x;
        }
        if (left.pixel.y != right.pixel.y)
        {
            return left.pixel.y < right.pixel.y;
        }
        return left.original_index < right.original_index;
    }

    static bool prepare_observed_points(
        const std::vector<cv::Point2f> &image_points_px,
        const cv::Matx33d &camera_matrix,
        const cv::Vec<double, 5> &distortion_coefficients,
        std::vector<ObservedPoint> &observed_points)
    {
        observed_points.clear();
        observed_points.reserve(image_points_px.size());
        for (size_t point_index = 0; point_index < image_points_px.size(); ++point_index)
        {
            ObservedPoint observed;
            observed.pixel = image_points_px[point_index];
            observed.original_index = static_cast<int>(point_index);
            observed_points.push_back(observed);
        }
        std::stable_sort(
            observed_points.begin(),
            observed_points.end(),
            observed_point_less);

        std::vector<cv::Point2f> sorted_pixels;
        sorted_pixels.reserve(observed_points.size());
        for (size_t point_index = 0; point_index < observed_points.size(); ++point_index)
        {
            sorted_pixels.push_back(observed_points[point_index].pixel);
        }

        std::vector<cv::Point2f> normalized_points;
        try
        {
            cv::undistortPoints(
                sorted_pixels,
                normalized_points,
                camera_matrix,
                distortion_coefficients);
        }
        catch (const cv::Exception &)
        {
            return false;
        }

        if (normalized_points.size() != observed_points.size())
        {
            return false;
        }

        for (size_t point_index = 0; point_index < observed_points.size(); ++point_index)
        {
            const cv::Point2f &normalized = normalized_points[point_index];
            if (!is_finite(normalized))
            {
                return false;
            }

            observed_points[point_index].normalized =
                cv::Point2d(normalized.x, normalized.y);
            cv::Vec3d ray(normalized.x, normalized.y, 1.);
            const double ray_length = cv::norm(ray);
            if (!std::isfinite(ray_length) || ray_length <= 1e-12)
            {
                return false;
            }
            observed_points[point_index].ray = ray * (1. / ray_length);
        }

        return true;
    }

    static bool build_orientation_hypotheses(
        const HMDPointCloudPosePrior &prior,
        const HMDPointCloudPoseSolverSettings &settings,
        const bool continuing,
        std::vector<OrientationHypothesis> &hypotheses)
    {
        hypotheses.clear();

        cv::Matx33d base_rotation;
        if (!rotation_from_rvec(prior.orientation_rvec, base_rotation))
        {
            return false;
        }

        const cv::Vec3d model_up(0., 1., 0.);
        cv::Vec3d camera_up = base_rotation * model_up;
        const double camera_up_length = cv::norm(camera_up);
        if (!std::isfinite(camera_up_length) || camera_up_length <= 1e-12)
        {
            return false;
        }
        camera_up *= 1. / camera_up_length;

        const int yaw_multipliers[HMDPointCloudPoseSolver::kMaxOrientationHypothesisCount] =
            {0, -1, 1, -2, 2};
        const int hypothesis_count =
            continuing ? 1 : HMDPointCloudPoseSolver::kMaxOrientationHypothesisCount;
        hypotheses.reserve(static_cast<size_t>(hypothesis_count));

        for (int hypothesis_index = 0;
             hypothesis_index < hypothesis_count;
             ++hypothesis_index)
        {
            const double yaw_degrees =
                static_cast<double>(yaw_multipliers[hypothesis_index]) *
                settings.acquisition_yaw_step_degrees;
            const cv::Vec3d yaw_rvec =
                camera_up * (yaw_degrees * CV_PI / 180.);

            cv::Matx33d yaw_rotation;
            if (!rotation_from_rvec(yaw_rvec, yaw_rotation))
            {
                return false;
            }

            OrientationHypothesis hypothesis;
            hypothesis.rotation = yaw_rotation * base_rotation;
            if (!rvec_from_rotation(hypothesis.rotation, hypothesis.rvec))
            {
                return false;
            }
            hypothesis.absolute_yaw_degrees = std::fabs(yaw_degrees);
            hypothesis.index = hypothesis_index;
            hypotheses.push_back(hypothesis);
        }

        return true;
    }

    static bool project_normalized(
        const std::vector<cv::Point3f> &model_points,
        const cv::Matx33d &rotation,
        const cv::Vec3d &position_cm,
        const double focal_length_px,
        const HMDPointCloudPoseSolverSettings &settings,
        std::vector<cv::Point2d> &projected_points,
        std::vector<double> &gates)
    {
        projected_points.clear();
        gates.clear();
        projected_points.reserve(model_points.size());
        gates.reserve(model_points.size());

        for (size_t point_index = 0; point_index < model_points.size(); ++point_index)
        {
            const cv::Vec3d camera_point =
                rotation * to_vec3d(model_points[point_index]) + position_cm;
            if (!is_finite(camera_point) ||
                camera_point[2] <= kMinimumPointDepthCm)
            {
                return false;
            }

            projected_points.push_back(
                cv::Point2d(
                    camera_point[0] / camera_point[2],
                    camera_point[1] / camera_point[2]));

            const double gate_px =
                clamp_double(
                    4. +
                        focal_length_px *
                            settings.coarse_geometry_tolerance_cm /
                            camera_point[2],
                    settings.min_association_distance_px,
                    settings.max_association_distance_px);
            gates.push_back(gate_px / focal_length_px);
        }

        return true;
    }

    static bool project_pixels(
        const std::vector<cv::Point3f> &model_points,
        const cv::Matx33d &rotation,
        const cv::Vec3d &orientation_rvec,
        const cv::Vec3d &position_cm,
        const cv::Matx33d &camera_matrix,
        const cv::Vec<double, 5> &distortion_coefficients,
        const double focal_length_px,
        const HMDPointCloudPoseSolverSettings &settings,
        std::vector<cv::Point2d> &projected_points,
        std::vector<double> &gates)
    {
        std::vector<cv::Point2f> projected_float;
        try
        {
            cv::projectPoints(
                model_points,
                orientation_rvec,
                position_cm,
                camera_matrix,
                distortion_coefficients,
                projected_float);
        }
        catch (const cv::Exception &)
        {
            return false;
        }

        if (projected_float.size() != model_points.size())
        {
            return false;
        }

        projected_points.clear();
        gates.clear();
        projected_points.reserve(model_points.size());
        gates.reserve(model_points.size());

        for (size_t point_index = 0; point_index < model_points.size(); ++point_index)
        {
            const cv::Vec3d camera_point =
                rotation * to_vec3d(model_points[point_index]) + position_cm;
            if (!is_finite(camera_point) ||
                camera_point[2] <= kMinimumPointDepthCm ||
                !is_finite(projected_float[point_index]))
            {
                return false;
            }

            projected_points.push_back(
                cv::Point2d(
                    projected_float[point_index].x,
                    projected_float[point_index].y));
            gates.push_back(
                clamp_double(
                    4. +
                        focal_length_px *
                            settings.strict_geometry_tolerance_cm /
                            camera_point[2],
                    settings.min_strict_gate_px,
                    settings.max_strict_gate_px));
        }

        return true;
    }

    static bool translation_is_plausible(
        const std::vector<cv::Point3f> &model_points,
        const cv::Matx33d &rotation,
        const cv::Vec3d &position_cm,
        const HMDPointCloudPoseSolverSettings &settings)
    {
        if (!is_finite(position_cm) ||
            std::fabs(position_cm[0]) > settings.max_lateral_position_cm ||
            std::fabs(position_cm[1]) > settings.max_lateral_position_cm)
        {
            return false;
        }

        std::vector<double> depths;
        depths.reserve(model_points.size());
        for (size_t point_index = 0; point_index < model_points.size(); ++point_index)
        {
            const cv::Vec3d camera_point =
                rotation * to_vec3d(model_points[point_index]) + position_cm;
            if (!is_finite(camera_point) ||
                camera_point[2] <= kMinimumPointDepthCm)
            {
                return false;
            }
            depths.push_back(camera_point[2]);
        }

        std::sort(depths.begin(), depths.end());
        const size_t middle = depths.size() / 2;
        const double median_depth =
            (depths.size() & 1) != 0
                ? depths[middle]
                : (depths[middle - 1] + depths[middle]) * 0.5;

        return
            median_depth >= settings.min_depth_cm &&
            median_depth <= settings.max_depth_cm;
    }

    static bool compute_coarse_score(
        const std::vector<cv::Point3f> &model_points,
        const std::vector<ObservedPoint> &observed_points,
        const OrientationHypothesis &orientation,
        const cv::Vec3d &position_cm,
        const double focal_length_px,
        const HMDPointCloudPoseSolverSettings &settings,
        int &match_count,
        double &normalized_squared_error)
    {
        std::vector<cv::Point2d> projected_points;
        std::vector<double> gates;
        if (!project_normalized(
                model_points,
                orientation.rotation,
                position_cm,
                focal_length_px,
                settings,
                projected_points,
                gates))
        {
            return false;
        }

        match_count = 0;
        normalized_squared_error = 0.;
        for (size_t model_index = 0;
             model_index < projected_points.size();
             ++model_index)
        {
            double nearest_normalized_error =
                std::numeric_limits<double>::max();
            const double inverse_gate_squared =
                1. / (gates[model_index] * gates[model_index]);

            for (size_t observed_index = 0;
                 observed_index < observed_points.size();
                 ++observed_index)
            {
                const double delta_x =
                    projected_points[model_index].x -
                    observed_points[observed_index].normalized.x;
                const double delta_y =
                    projected_points[model_index].y -
                    observed_points[observed_index].normalized.y;
                const double normalized_error =
                    (delta_x * delta_x + delta_y * delta_y) *
                    inverse_gate_squared;
                nearest_normalized_error =
                    std::min(nearest_normalized_error, normalized_error);
            }

            if (nearest_normalized_error <= 1.)
            {
                ++match_count;
                normalized_squared_error += nearest_normalized_error;
            }
        }

        return match_count >= 3;
    }

    static bool candidate_less(
        const TranslationCandidate &left,
        const TranslationCandidate &right)
    {
        if (left.coarse_match_count != right.coarse_match_count)
        {
            return left.coarse_match_count > right.coarse_match_count;
        }
        if (left.coarse_normalized_squared_error !=
            right.coarse_normalized_squared_error)
        {
            return
                left.coarse_normalized_squared_error <
                right.coarse_normalized_squared_error;
        }
        if (left.pair_residual_cm != right.pair_residual_cm)
        {
            return left.pair_residual_cm < right.pair_residual_cm;
        }
        if (left.prior_distance_cm != right.prior_distance_cm)
        {
            return left.prior_distance_cm < right.prior_distance_cm;
        }
        if (left.absolute_yaw_degrees != right.absolute_yaw_degrees)
        {
            return left.absolute_yaw_degrees < right.absolute_yaw_degrees;
        }
        if (left.orientation_index != right.orientation_index)
        {
            return left.orientation_index < right.orientation_index;
        }
        if (left.position_cm[0] != right.position_cm[0])
        {
            return left.position_cm[0] < right.position_cm[0];
        }
        if (left.position_cm[1] != right.position_cm[1])
        {
            return left.position_cm[1] < right.position_cm[1];
        }
        if (left.position_cm[2] != right.position_cm[2])
        {
            return left.position_cm[2] < right.position_cm[2];
        }
        if (left.model_index_a != right.model_index_a)
        {
            return left.model_index_a < right.model_index_a;
        }
        if (left.model_index_b != right.model_index_b)
        {
            return left.model_index_b < right.model_index_b;
        }
        if (left.observed_index_a != right.observed_index_a)
        {
            return left.observed_index_a < right.observed_index_a;
        }
        return left.observed_index_b < right.observed_index_b;
    }

    static bool add_scored_candidate(
        const std::vector<cv::Point3f> &model_points,
        const std::vector<ObservedPoint> &observed_points,
        const std::vector<OrientationHypothesis> &orientations,
        const HMDPointCloudPosePrior &prior,
        const HMDPointCloudPoseSolverSettings &settings,
        const double focal_length_px,
        TranslationCandidate candidate,
        std::vector<TranslationCandidate> &candidates,
        HMDPointCloudPoseSolverDiagnostics &diagnostics)
    {
        const OrientationHypothesis &orientation =
            orientations[candidate.orientation_index];
        if (!translation_is_plausible(
                model_points,
                orientation.rotation,
                candidate.position_cm,
                settings))
        {
            return false;
        }

        if (prior.has_position)
        {
            candidate.prior_distance_cm =
                cv::norm(candidate.position_cm - prior.position_cm);
        }

        ++diagnostics.coarse_hypothesis_count;
        if (!compute_coarse_score(
                model_points,
                observed_points,
                orientation,
                candidate.position_cm,
                focal_length_px,
                settings,
                candidate.coarse_match_count,
                candidate.coarse_normalized_squared_error))
        {
            return false;
        }

        candidates.push_back(candidate);
        return true;
    }

    static bool solve_two_ray_translation(
        const cv::Vec3d &model_delta_camera,
        const cv::Vec3d &model_point_a_camera,
        const cv::Vec3d &model_point_b_camera,
        const cv::Vec3d &ray_a,
        const cv::Vec3d &ray_b,
        const double minimum_determinant,
        cv::Vec3d &translation_cm,
        double &residual_cm,
        double &lambda_a,
        double &lambda_b)
    {
        const double ray_dot =
            clamp_double(ray_a.dot(ray_b), -1., 1.);
        const double determinant = 1. - ray_dot * ray_dot;
        if (!std::isfinite(determinant) ||
            determinant < minimum_determinant)
        {
            return false;
        }

        const double rhs_a = ray_a.dot(model_delta_camera);
        const double rhs_b = -ray_b.dot(model_delta_camera);
        lambda_a = (rhs_a + ray_dot * rhs_b) / determinant;
        lambda_b = (ray_dot * rhs_a + rhs_b) / determinant;
        if (!std::isfinite(lambda_a) || !std::isfinite(lambda_b))
        {
            return false;
        }

        const cv::Vec3d translation_a =
            ray_a * lambda_a - model_point_a_camera;
        const cv::Vec3d translation_b =
            ray_b * lambda_b - model_point_b_camera;
        translation_cm = (translation_a + translation_b) * 0.5;
        residual_cm = cv::norm(translation_a - translation_b);

        return
            is_finite(translation_cm) &&
            std::isfinite(residual_cm);
    }

    static void generate_translation_candidates(
        const std::vector<cv::Point3f> &model_points,
        const std::vector<ObservedPoint> &observed_points,
        const std::vector<OrientationHypothesis> &orientations,
        const HMDPointCloudPosePrior &prior,
        const HMDPointCloudPoseSolverSettings &settings,
        const double focal_length_px,
        std::vector<TranslationCandidate> &candidates,
        HMDPointCloudPoseSolverDiagnostics &diagnostics)
    {
        candidates.clear();

        const double minimum_ray_angle_radians =
            settings.min_ray_angle_degrees * CV_PI / 180.;
        const double minimum_determinant =
            std::sin(minimum_ray_angle_radians) *
            std::sin(minimum_ray_angle_radians);
        const double minimum_blob_distance_squared =
            settings.min_blob_pair_distance_px *
            settings.min_blob_pair_distance_px;

        for (size_t orientation_index = 0;
             orientation_index < orientations.size();
             ++orientation_index)
        {
            const OrientationHypothesis &orientation =
                orientations[orientation_index];

            if (prior.has_position && is_finite(prior.position_cm))
            {
                TranslationCandidate prior_candidate;
                prior_candidate.orientation_index =
                    static_cast<int>(orientation_index);
                prior_candidate.position_cm = prior.position_cm;
                prior_candidate.absolute_yaw_degrees =
                    orientation.absolute_yaw_degrees;
                add_scored_candidate(
                    model_points,
                    observed_points,
                    orientations,
                    prior,
                    settings,
                    focal_length_px,
                    prior_candidate,
                    candidates,
                    diagnostics);
            }

            for (size_t model_index_a = 0;
                 model_index_a + 1 < model_points.size();
                 ++model_index_a)
            {
                for (size_t model_index_b = model_index_a + 1;
                     model_index_b < model_points.size();
                     ++model_index_b)
                {
                    const cv::Vec3d model_point_a =
                        to_vec3d(model_points[model_index_a]);
                    const cv::Vec3d model_point_b =
                        to_vec3d(model_points[model_index_b]);
                    if (cv::norm(model_point_a - model_point_b) <
                        settings.min_model_pair_distance_cm)
                    {
                        continue;
                    }

                    const cv::Vec3d model_point_a_camera =
                        orientation.rotation * model_point_a;
                    const cv::Vec3d model_point_b_camera =
                        orientation.rotation * model_point_b;
                    const cv::Vec3d model_delta_camera =
                        model_point_a_camera - model_point_b_camera;

                    for (size_t observed_index_a = 0;
                         observed_index_a + 1 < observed_points.size();
                         ++observed_index_a)
                    {
                        for (size_t observed_index_b = observed_index_a + 1;
                             observed_index_b < observed_points.size();
                             ++observed_index_b)
                        {
                            const cv::Point2f pixel_delta =
                                observed_points[observed_index_a].pixel -
                                observed_points[observed_index_b].pixel;
                            const double pixel_distance_squared =
                                static_cast<double>(pixel_delta.x) *
                                    static_cast<double>(pixel_delta.x) +
                                static_cast<double>(pixel_delta.y) *
                                    static_cast<double>(pixel_delta.y);
                            if (pixel_distance_squared <
                                minimum_blob_distance_squared)
                            {
                                continue;
                            }

                            for (int ordering = 0; ordering < 2; ++ordering)
                            {
                                if (diagnostics.pair_hypothesis_count >=
                                    HMDPointCloudPoseSolver::
                                        kMaxRawPairHypothesisCount)
                                {
                                    return;
                                }
                                ++diagnostics.pair_hypothesis_count;

                                const size_t ray_index_a =
                                    ordering == 0
                                        ? observed_index_a
                                        : observed_index_b;
                                const size_t ray_index_b =
                                    ordering == 0
                                        ? observed_index_b
                                        : observed_index_a;

                                cv::Vec3d position_cm;
                                double residual_cm = 0.;
                                double lambda_a = 0.;
                                double lambda_b = 0.;
                                if (!solve_two_ray_translation(
                                        model_delta_camera,
                                        model_point_a_camera,
                                        model_point_b_camera,
                                        observed_points[ray_index_a].ray,
                                        observed_points[ray_index_b].ray,
                                        minimum_determinant,
                                        position_cm,
                                        residual_cm,
                                        lambda_a,
                                        lambda_b) ||
                                    lambda_a < settings.min_depth_cm ||
                                    lambda_a > settings.max_depth_cm ||
                                    lambda_b < settings.min_depth_cm ||
                                    lambda_b > settings.max_depth_cm ||
                                    residual_cm >
                                        settings.
                                            max_pair_translation_residual_cm)
                                {
                                    continue;
                                }

                                ++diagnostics.accepted_pair_hypothesis_count;
                                TranslationCandidate candidate;
                                candidate.orientation_index =
                                    static_cast<int>(orientation_index);
                                candidate.position_cm = position_cm;
                                candidate.pair_residual_cm = residual_cm;
                                candidate.absolute_yaw_degrees =
                                    orientation.absolute_yaw_degrees;
                                candidate.model_index_a =
                                    static_cast<int>(model_index_a);
                                candidate.model_index_b =
                                    static_cast<int>(model_index_b);
                                candidate.observed_index_a =
                                    static_cast<int>(ray_index_a);
                                candidate.observed_index_b =
                                    static_cast<int>(ray_index_b);
                                add_scored_candidate(
                                    model_points,
                                    observed_points,
                                    orientations,
                                    prior,
                                    settings,
                                    focal_length_px,
                                    candidate,
                                    candidates,
                                    diagnostics);
                            }
                        }
                    }
                }
            }
        }
    }

    static void retain_coarse_candidates(
        std::vector<TranslationCandidate> &candidates,
        HMDPointCloudPoseSolverDiagnostics &diagnostics)
    {
        std::stable_sort(
            candidates.begin(),
            candidates.end(),
            candidate_less);

        typedef std::tuple<int, int, int, int> TranslationCell;
        std::set<TranslationCell> occupied_cells;
        std::vector<TranslationCandidate> retained;
        retained.reserve(
            HMDPointCloudPoseSolver::kMaxCoarseCandidateCount);

        for (size_t candidate_index = 0;
             candidate_index < candidates.size() &&
             retained.size() <
                 static_cast<size_t>(
                     HMDPointCloudPoseSolver::kMaxCoarseCandidateCount);
             ++candidate_index)
        {
            const TranslationCandidate &candidate =
                candidates[candidate_index];
            const TranslationCell cell(
                candidate.orientation_index,
                static_cast<int>(
                    std::floor(
                        candidate.position_cm[0] /
                        kTranslationQuantizationCm)),
                static_cast<int>(
                    std::floor(
                        candidate.position_cm[1] /
                        kTranslationQuantizationCm)),
                static_cast<int>(
                    std::floor(
                        candidate.position_cm[2] /
                        kTranslationQuantizationCm)));
            if (occupied_cells.insert(cell).second)
            {
                retained.push_back(candidate);
            }
        }

        candidates.swap(retained);
        diagnostics.retained_coarse_candidate_count =
            static_cast<int>(candidates.size());
    }

    static bool solve_hungarian_assignment(
        const std::vector<cv::Point2d> &projected_points,
        const std::vector<cv::Point2d> &observed_points,
        const std::vector<double> &gates,
        AssignmentResult &result)
    {
        result = AssignmentResult();

        const int model_count =
            static_cast<int>(projected_points.size());
        const int observed_count =
            static_cast<int>(observed_points.size());
        if (model_count <= 0 ||
            observed_count <= 0 ||
            gates.size() != projected_points.size())
        {
            return false;
        }

        const int column_count = observed_count + model_count;
        std::vector<std::vector<double> > costs(
            static_cast<size_t>(model_count),
            std::vector<double>(
                static_cast<size_t>(column_count),
                kInvalidAssignmentCost));

        for (int model_index = 0;
             model_index < model_count;
             ++model_index)
        {
            const double gate = gates[model_index];
            const double inverse_gate_squared = 1. / (gate * gate);
            for (int observed_index = 0;
                 observed_index < observed_count;
                 ++observed_index)
            {
                const double delta_x =
                    projected_points[model_index].x -
                    observed_points[observed_index].x;
                const double delta_y =
                    projected_points[model_index].y -
                    observed_points[observed_index].y;
                const double normalized_cost =
                    (delta_x * delta_x + delta_y * delta_y) *
                    inverse_gate_squared;
                if (normalized_cost <= 1.)
                {
                    costs[model_index][observed_index] =
                        normalized_cost;
                }
            }
            costs[model_index][observed_count + model_index] =
                kUnmatchedAssignmentCost;
        }

        // Rectangular Hungarian minimization, 1-indexed internally. Iterating
        // columns in ascending order is the deterministic tie breaker.
        std::vector<double> row_potential(
            static_cast<size_t>(model_count + 1), 0.);
        std::vector<double> column_potential(
            static_cast<size_t>(column_count + 1), 0.);
        std::vector<int> column_row(
            static_cast<size_t>(column_count + 1), 0);
        std::vector<int> previous_column(
            static_cast<size_t>(column_count + 1), 0);

        for (int row = 1; row <= model_count; ++row)
        {
            column_row[0] = row;
            int current_column = 0;
            std::vector<double> minimum_value(
                static_cast<size_t>(column_count + 1),
                std::numeric_limits<double>::max());
            std::vector<unsigned char> used(
                static_cast<size_t>(column_count + 1), 0);

            do
            {
                used[current_column] = 1;
                const int current_row = column_row[current_column];
                double delta = std::numeric_limits<double>::max();
                int next_column = 0;

                for (int column = 1;
                     column <= column_count;
                     ++column)
                {
                    if (used[column] != 0)
                    {
                        continue;
                    }

                    const double reduced_cost =
                        costs[current_row - 1][column - 1] -
                        row_potential[current_row] -
                        column_potential[column];
                    if (reduced_cost < minimum_value[column])
                    {
                        minimum_value[column] = reduced_cost;
                        previous_column[column] = current_column;
                    }
                    if (minimum_value[column] < delta)
                    {
                        delta = minimum_value[column];
                        next_column = column;
                    }
                }

                if (!std::isfinite(delta) || next_column == 0)
                {
                    return false;
                }

                for (int column = 0;
                     column <= column_count;
                     ++column)
                {
                    if (used[column] != 0)
                    {
                        row_potential[column_row[column]] += delta;
                        column_potential[column] -= delta;
                    }
                    else
                    {
                        minimum_value[column] -= delta;
                    }
                }
                current_column = next_column;
            }
            while (column_row[current_column] != 0);

            do
            {
                const int prior_column =
                    previous_column[current_column];
                column_row[current_column] =
                    column_row[prior_column];
                current_column = prior_column;
            }
            while (current_column != 0);
        }

        std::vector<int> row_column(
            static_cast<size_t>(model_count), -1);
        for (int column = 1;
             column <= column_count;
             ++column)
        {
            if (column_row[column] > 0)
            {
                row_column[column_row[column] - 1] = column - 1;
            }
        }

        for (int model_index = 0;
             model_index < model_count;
             ++model_index)
        {
            const int column = row_column[model_index];
            if (column >= 0 &&
                column < observed_count &&
                costs[model_index][column] <= 1.)
            {
                result.pairs.push_back(
                    std::make_pair(model_index, column));
                result.normalized_squared_error +=
                    costs[model_index][column];
            }
        }
        result.match_count =
            static_cast<int>(result.pairs.size());

        return result.match_count > 0;
    }

    static bool mapping_less(
        const MappingHypothesis &left,
        const MappingHypothesis &right)
    {
        if (left.assignment.match_count !=
            right.assignment.match_count)
        {
            return
                left.assignment.match_count >
                right.assignment.match_count;
        }

        const double left_mean_error =
            left.assignment.normalized_squared_error /
            static_cast<double>(left.assignment.match_count);
        const double right_mean_error =
            right.assignment.normalized_squared_error /
            static_cast<double>(right.assignment.match_count);
        if (left_mean_error != right_mean_error)
        {
            return left_mean_error < right_mean_error;
        }
        return candidate_less(left.candidate, right.candidate);
    }

    static void build_mapping_hypotheses(
        const std::vector<cv::Point3f> &model_points,
        const std::vector<ObservedPoint> &observed_points,
        const std::vector<OrientationHypothesis> &orientations,
        const std::vector<TranslationCandidate> &candidates,
        const double focal_length_px,
        const HMDPointCloudPoseSolverSettings &settings,
        std::vector<MappingHypothesis> &mappings,
        HMDPointCloudPoseSolverDiagnostics &diagnostics)
    {
        std::vector<cv::Point2d> observed_normalized;
        observed_normalized.reserve(observed_points.size());
        for (size_t observed_index = 0;
             observed_index < observed_points.size();
             ++observed_index)
        {
            observed_normalized.push_back(
                observed_points[observed_index].normalized);
        }

        mappings.clear();
        for (size_t candidate_index = 0;
             candidate_index < candidates.size();
             ++candidate_index)
        {
            const TranslationCandidate &candidate =
                candidates[candidate_index];
            const OrientationHypothesis &orientation =
                orientations[candidate.orientation_index];

            std::vector<cv::Point2d> projected_points;
            std::vector<double> gates;
            if (!project_normalized(
                    model_points,
                    orientation.rotation,
                    candidate.position_cm,
                    focal_length_px,
                    settings,
                    projected_points,
                    gates))
            {
                continue;
            }

            ++diagnostics.exact_assignment_count;
            AssignmentResult assignment;
            if (solve_hungarian_assignment(
                    projected_points,
                    observed_normalized,
                    gates,
                    assignment) &&
                assignment.match_count >= 4)
            {
                MappingHypothesis mapping;
                mapping.candidate = candidate;
                mapping.assignment = assignment;
                mappings.push_back(mapping);
            }
        }

        std::stable_sort(
            mappings.begin(),
            mappings.end(),
            mapping_less);

        std::set<std::vector<std::pair<int, int> > > seen_mappings;
        std::vector<MappingHypothesis> unique_mappings;
        unique_mappings.reserve(
            HMDPointCloudPoseSolver::kMaxMappingHypothesisCount);
        for (size_t mapping_index = 0;
             mapping_index < mappings.size() &&
             unique_mappings.size() <
                 static_cast<size_t>(
                     HMDPointCloudPoseSolver::
                         kMaxMappingHypothesisCount);
             ++mapping_index)
        {
            if (seen_mappings.insert(
                    mappings[mapping_index].assignment.pairs).second)
            {
                unique_mappings.push_back(mappings[mapping_index]);
            }
        }

        mappings.swap(unique_mappings);
        diagnostics.unique_mapping_count =
            static_cast<int>(mappings.size());
    }

    static void generate_combinations_recursive(
        const int item_count,
        const int combination_size,
        const int next_index,
        std::vector<int> &current,
        std::vector<std::vector<int> > &combinations)
    {
        if (static_cast<int>(current.size()) == combination_size)
        {
            combinations.push_back(current);
            return;
        }

        const int remaining_needed =
            combination_size - static_cast<int>(current.size());
        for (int index = next_index;
             index <= item_count - remaining_needed;
             ++index)
        {
            current.push_back(index);
            generate_combinations_recursive(
                item_count,
                combination_size,
                index + 1,
                current,
                combinations);
            current.pop_back();
        }
    }

    static void generate_seed_subsets(
        const int match_count,
        std::vector<std::vector<int> > &subsets)
    {
        subsets.clear();
        std::vector<int> full;
        full.reserve(static_cast<size_t>(match_count));
        for (int index = 0; index < match_count; ++index)
        {
            full.push_back(index);
        }
        subsets.push_back(full);

        const int subset_size = std::max(4, match_count - 2);
        if (subset_size < match_count)
        {
            std::vector<int> current;
            current.reserve(static_cast<size_t>(subset_size));
            generate_combinations_recursive(
                match_count,
                subset_size,
                0,
                current,
                subsets);
        }
    }

    static bool call_seeded_iterative_pnp(
        const std::vector<cv::Point3f> &model_points,
        const std::vector<ObservedPoint> &observed_points,
        const std::vector<std::pair<int, int> > &assignments,
        const std::vector<int> &assignment_subset,
        const cv::Matx33d &camera_matrix,
        const cv::Vec<double, 5> &distortion_coefficients,
        const cv::Vec3d &initial_orientation_rvec,
        const cv::Vec3d &initial_position_cm,
        cv::Vec3d &solved_orientation_rvec,
        cv::Vec3d &solved_position_cm,
        HMDPointCloudPoseSolverDiagnostics &diagnostics)
    {
        if (assignment_subset.size() < 4 ||
            diagnostics.pnp_attempt_count >=
                HMDPointCloudPoseSolver::kMaxPnPAttemptCount)
        {
            return false;
        }
        ++diagnostics.pnp_attempt_count;

        std::vector<cv::Point3f> subset_model_points;
        std::vector<cv::Point2f> subset_image_points;
        subset_model_points.reserve(assignment_subset.size());
        subset_image_points.reserve(assignment_subset.size());
        for (size_t subset_index = 0;
             subset_index < assignment_subset.size();
             ++subset_index)
        {
            const int assignment_index =
                assignment_subset[subset_index];
            if (assignment_index < 0 ||
                assignment_index >=
                    static_cast<int>(assignments.size()))
            {
                return false;
            }

            const std::pair<int, int> &assignment =
                assignments[assignment_index];
            subset_model_points.push_back(
                model_points[assignment.first]);
            subset_image_points.push_back(
                observed_points[assignment.second].pixel);
        }

        cv::Mat rvec(3, 1, CV_64F);
        cv::Mat tvec(3, 1, CV_64F);
        for (int axis = 0; axis < 3; ++axis)
        {
            rvec.at<double>(axis, 0) =
                initial_orientation_rvec[axis];
            tvec.at<double>(axis, 0) =
                initial_position_cm[axis];
        }

        bool solved = false;
        try
        {
            solved = cv::solvePnP(
                subset_model_points,
                subset_image_points,
                camera_matrix,
                distortion_coefficients,
                rvec,
                tvec,
                true,
                cv::SOLVEPNP_ITERATIVE);
        }
        catch (const cv::Exception &)
        {
            return false;
        }

        if (!solved)
        {
            return false;
        }

        solved_orientation_rvec = cv::Vec3d(
            rvec.at<double>(0, 0),
            rvec.at<double>(1, 0),
            rvec.at<double>(2, 0));
        solved_position_cm = cv::Vec3d(
            tvec.at<double>(0, 0),
            tvec.at<double>(1, 0),
            tvec.at<double>(2, 0));
        if (!is_finite(solved_orientation_rvec) ||
            !is_finite(solved_position_cm))
        {
            return false;
        }

        cv::Matx33d solved_rotation;
        if (!rotation_from_rvec(
                solved_orientation_rvec,
                solved_rotation))
        {
            return false;
        }

        ++diagnostics.successful_pnp_count;
        return true;
    }

    static double convex_hull_area(
        const std::vector<cv::Point2f> &points)
    {
        if (points.size() < 3)
        {
            return 0.;
        }

        std::vector<cv::Point2f> hull;
        cv::convexHull(points, hull);
        return std::fabs(cv::contourArea(hull));
    }

    static bool evaluate_pose(
        const std::vector<cv::Point3f> &model_points,
        const std::vector<ObservedPoint> &observed_points,
        const cv::Matx33d &camera_matrix,
        const cv::Vec<double, 5> &distortion_coefficients,
        const HMDPointCloudPosePrior &prior,
        const HMDPointCloudPoseSolverSettings &settings,
        const bool continuing,
        const double focal_length_px,
        const cv::Vec3d &orientation_rvec,
        const cv::Vec3d &position_cm,
        PoseCandidate &pose)
    {
        cv::Matx33d orientation;
        cv::Matx33d prior_orientation;
        if (!rotation_from_rvec(orientation_rvec, orientation) ||
            !rotation_from_rvec(
                prior.orientation_rvec,
                prior_orientation) ||
            !translation_is_plausible(
                model_points,
                orientation,
                position_cm,
                settings))
        {
            return false;
        }

        std::vector<cv::Point2d> projected_points;
        std::vector<double> gates;
        if (!project_pixels(
                model_points,
                orientation,
                orientation_rvec,
                position_cm,
                camera_matrix,
                distortion_coefficients,
                focal_length_px,
                settings,
                projected_points,
                gates))
        {
            return false;
        }

        std::vector<cv::Point2d> observed_pixels;
        observed_pixels.reserve(observed_points.size());
        for (size_t observed_index = 0;
             observed_index < observed_points.size();
             ++observed_index)
        {
            observed_pixels.push_back(
                cv::Point2d(
                    observed_points[observed_index].pixel.x,
                    observed_points[observed_index].pixel.y));
        }

        AssignmentResult assignment;
        if (!solve_hungarian_assignment(
                projected_points,
                observed_pixels,
                gates,
                assignment))
        {
            return false;
        }

        const int required_match_count =
            continuing
                ? settings.min_continuation_inlier_count
                : settings.min_inlier_count;
        if (assignment.match_count < required_match_count)
        {
            return false;
        }

        double squared_pixel_error = 0.;
        double squared_normalized_error = 0.;
        double maximum_normalized_error = 0.;
        std::vector<cv::Point2f> matched_projected_points;
        matched_projected_points.reserve(assignment.pairs.size());
        for (size_t assignment_index = 0;
             assignment_index < assignment.pairs.size();
             ++assignment_index)
        {
            const int model_index =
                assignment.pairs[assignment_index].first;
            const int observed_index =
                assignment.pairs[assignment_index].second;
            const double delta_x =
                projected_points[model_index].x -
                observed_points[observed_index].pixel.x;
            const double delta_y =
                projected_points[model_index].y -
                observed_points[observed_index].pixel.y;
            const double pixel_error_squared =
                delta_x * delta_x + delta_y * delta_y;
            const double normalized_error =
                std::sqrt(pixel_error_squared) /
                gates[model_index];

            squared_pixel_error += pixel_error_squared;
            squared_normalized_error +=
                normalized_error * normalized_error;
            maximum_normalized_error =
                std::max(
                    maximum_normalized_error,
                    normalized_error);
            matched_projected_points.push_back(
                cv::Point2f(
                    static_cast<float>(
                        projected_points[model_index].x),
                    static_cast<float>(
                        projected_points[model_index].y)));
        }

        const double inverse_match_count =
            1. / static_cast<double>(assignment.match_count);
        const double reprojection_error_px =
            std::sqrt(squared_pixel_error * inverse_match_count);
        const double normalized_rms_error =
            std::sqrt(
                squared_normalized_error *
                inverse_match_count);
        if (!std::isfinite(reprojection_error_px) ||
            !std::isfinite(normalized_rms_error) ||
            reprojection_error_px >
                settings.max_reprojection_error_px ||
            normalized_rms_error >
                settings.max_normalized_rms_error ||
            maximum_normalized_error > 1.)
        {
            return false;
        }

        std::vector<cv::Point2f> all_projected_points;
        all_projected_points.reserve(projected_points.size());
        for (size_t point_index = 0;
             point_index < projected_points.size();
             ++point_index)
        {
            all_projected_points.push_back(
                cv::Point2f(
                    static_cast<float>(
                        projected_points[point_index].x),
                    static_cast<float>(
                        projected_points[point_index].y)));
        }

        const double full_projection_area =
            convex_hull_area(all_projected_points);
        const double matched_projection_area =
            convex_hull_area(matched_projected_points);
        if (!std::isfinite(full_projection_area) ||
            !std::isfinite(matched_projection_area) ||
            full_projection_area <
                settings.min_projection_area_px_sqr ||
            matched_projection_area <= 1.)
        {
            return false;
        }

        const double projection_coverage =
            clamp_double(
                matched_projection_area /
                    full_projection_area,
                0.,
                1.);
        const double required_coverage =
            continuing
                ? settings.min_continuation_coverage
                : settings.min_acquisition_coverage;
        if (projection_coverage < required_coverage)
        {
            return false;
        }

        const double total_orientation_error_degrees =
            rotation_error_degrees(
                orientation,
                prior_orientation);
        double tilt_error_degrees = 0.;
        double heading_error_degrees = 0.;
        compute_tilt_and_heading_errors(
            orientation,
            prior_orientation,
            tilt_error_degrees,
            heading_error_degrees);

        const double tilt_gate_degrees =
            continuing
                ? settings.
                      max_continuation_tilt_error_degrees
                : settings.
                      max_acquisition_tilt_error_degrees;
        const double heading_gate_degrees =
            continuing
                ? settings.
                      max_continuation_heading_error_degrees
                : settings.
                      max_acquisition_heading_error_degrees;
        if (!std::isfinite(total_orientation_error_degrees) ||
            !std::isfinite(tilt_error_degrees) ||
            !std::isfinite(heading_error_degrees) ||
            total_orientation_error_degrees >
                settings.max_orientation_error_degrees ||
            tilt_error_degrees > tilt_gate_degrees ||
            heading_error_degrees > heading_gate_degrees)
        {
            return false;
        }

        double translation_error_cm = 0.;
        double translation_gate_cm =
            settings.max_translation_jump_cm;
        if (continuing)
        {
            const double prior_age_seconds =
                std::max(0., prior.position_age_seconds);
            translation_gate_cm =
                std::min(
                    settings.max_translation_jump_cm,
                    settings.base_translation_gate_cm +
                        settings.
                            translation_gate_speed_cm_per_second *
                            prior_age_seconds);
            translation_error_cm =
                cv::norm(position_cm - prior.position_cm);
            if (!std::isfinite(translation_error_cm) ||
                translation_error_cm > translation_gate_cm)
            {
                return false;
            }
        }

        const double tilt_normalized =
            tilt_error_degrees / tilt_gate_degrees;
        const double heading_normalized =
            heading_error_degrees / heading_gate_degrees;
        const double translation_normalized =
            continuing
                ? translation_error_cm / translation_gate_cm
                : 0.;

        pose.orientation_rvec = orientation_rvec;
        pose.position_cm = position_cm;
        pose.assignments = assignment.pairs;
        pose.match_count = assignment.match_count;
        pose.reprojection_error_px =
            reprojection_error_px;
        pose.maximum_normalized_error =
            maximum_normalized_error;
        pose.normalized_rms_error =
            normalized_rms_error;
        pose.projection_area_px_sqr =
            matched_projection_area;
        pose.projection_coverage =
            projection_coverage;
        pose.total_orientation_error_degrees =
            total_orientation_error_degrees;
        pose.tilt_error_degrees =
            tilt_error_degrees;
        pose.heading_error_degrees =
            heading_error_degrees;
        pose.translation_error_cm =
            translation_error_cm;
        pose.score =
            1000. * static_cast<double>(pose.match_count) +
            100. * pose.projection_coverage -
            100. *
                std::min(
                    4.,
                    pose.normalized_rms_error *
                        pose.normalized_rms_error) -
            40. *
                std::min(
                    4.,
                    pose.maximum_normalized_error *
                        pose.maximum_normalized_error) -
            20. *
                std::min(
                    4.,
                    tilt_normalized * tilt_normalized) -
            10. *
                std::min(
                    4.,
                    heading_normalized * heading_normalized) -
            10. *
                std::min(
                    4.,
                    translation_normalized *
                        translation_normalized);

        return std::isfinite(pose.score);
    }

    static bool pose_less(
        const PoseCandidate &left,
        const PoseCandidate &right)
    {
        if (left.score != right.score)
        {
            return left.score > right.score;
        }
        if (left.match_count != right.match_count)
        {
            return left.match_count > right.match_count;
        }
        if (left.reprojection_error_px !=
            right.reprojection_error_px)
        {
            return
                left.reprojection_error_px <
                right.reprojection_error_px;
        }
        if (left.orientation_rvec[0] !=
            right.orientation_rvec[0])
        {
            return
                left.orientation_rvec[0] <
                right.orientation_rvec[0];
        }
        if (left.orientation_rvec[1] !=
            right.orientation_rvec[1])
        {
            return
                left.orientation_rvec[1] <
                right.orientation_rvec[1];
        }
        if (left.orientation_rvec[2] !=
            right.orientation_rvec[2])
        {
            return
                left.orientation_rvec[2] <
                right.orientation_rvec[2];
        }
        if (left.position_cm[0] != right.position_cm[0])
        {
            return left.position_cm[0] < right.position_cm[0];
        }
        if (left.position_cm[1] != right.position_cm[1])
        {
            return left.position_cm[1] < right.position_cm[1];
        }
        return left.position_cm[2] < right.position_cm[2];
    }

    static bool poses_share_cluster(
        const PoseCandidate &left,
        const PoseCandidate &right)
    {
        if (cv::norm(left.position_cm - right.position_cm) >=
            kPoseClusterTranslationCm)
        {
            return false;
        }

        cv::Matx33d left_rotation;
        cv::Matx33d right_rotation;
        return
            rotation_from_rvec(
                left.orientation_rvec,
                left_rotation) &&
            rotation_from_rvec(
                right.orientation_rvec,
                right_rotation) &&
            rotation_error_degrees(
                left_rotation,
                right_rotation) <
                kPoseClusterRotationDegrees;
    }

    static void retain_pose_clusters(
        std::vector<PoseCandidate> &poses)
    {
        std::stable_sort(
            poses.begin(),
            poses.end(),
            pose_less);

        std::vector<PoseCandidate> retained;
        retained.reserve(
            HMDPointCloudPoseSolver::kMaxPoseCandidateCount);
        for (size_t pose_index = 0;
             pose_index < poses.size() &&
             retained.size() <
                 static_cast<size_t>(
                     HMDPointCloudPoseSolver::
                         kMaxPoseCandidateCount);
             ++pose_index)
        {
            bool duplicate = false;
            for (size_t retained_index = 0;
                 retained_index < retained.size();
                 ++retained_index)
            {
                if (poses_share_cluster(
                        poses[pose_index],
                        retained[retained_index]))
                {
                    duplicate = true;
                    break;
                }
            }
            if (!duplicate)
            {
                retained.push_back(poses[pose_index]);
            }
        }

        poses.swap(retained);
    }

    static void generate_pose_candidates(
        const std::vector<cv::Point3f> &model_points,
        const std::vector<ObservedPoint> &observed_points,
        const std::vector<OrientationHypothesis> &orientations,
        const std::vector<MappingHypothesis> &mappings,
        const cv::Matx33d &camera_matrix,
        const cv::Vec<double, 5> &distortion_coefficients,
        const HMDPointCloudPosePrior &prior,
        const HMDPointCloudPoseSolverSettings &settings,
        const bool continuing,
        const double focal_length_px,
        std::vector<PoseCandidate> &poses,
        HMDPointCloudPoseSolverDiagnostics &diagnostics)
    {
        poses.clear();

        for (size_t mapping_index = 0;
             mapping_index < mappings.size();
             ++mapping_index)
        {
            const MappingHypothesis &mapping =
                mappings[mapping_index];
            const OrientationHypothesis &orientation =
                orientations[
                    mapping.candidate.orientation_index];

            std::vector<std::vector<int> > subsets;
            generate_seed_subsets(
                mapping.assignment.match_count,
                subsets);
            for (size_t subset_index = 0;
                 subset_index < subsets.size();
                 ++subset_index)
            {
                cv::Vec3d solved_orientation;
                cv::Vec3d solved_position;
                if (!call_seeded_iterative_pnp(
                        model_points,
                        observed_points,
                        mapping.assignment.pairs,
                        subsets[subset_index],
                        camera_matrix,
                        distortion_coefficients,
                        orientation.rvec,
                        mapping.candidate.position_cm,
                        solved_orientation,
                        solved_position,
                        diagnostics))
                {
                    continue;
                }

                PoseCandidate pose;
                if (evaluate_pose(
                        model_points,
                        observed_points,
                        camera_matrix,
                        distortion_coefficients,
                        prior,
                        settings,
                        continuing,
                        focal_length_px,
                        solved_orientation,
                        solved_position,
                        pose))
                {
                    poses.push_back(pose);
                }
            }
        }

        retain_pose_clusters(poses);
        diagnostics.retained_pose_candidate_count =
            static_cast<int>(poses.size());
    }

    static void refine_pose_candidates(
        const std::vector<cv::Point3f> &model_points,
        const std::vector<ObservedPoint> &observed_points,
        const cv::Matx33d &camera_matrix,
        const cv::Vec<double, 5> &distortion_coefficients,
        const HMDPointCloudPosePrior &prior,
        const HMDPointCloudPoseSolverSettings &settings,
        const bool continuing,
        const double focal_length_px,
        std::vector<PoseCandidate> &poses,
        HMDPointCloudPoseSolverDiagnostics &diagnostics)
    {
        std::vector<PoseCandidate> refined_poses;
        refined_poses.reserve(poses.size());

        for (size_t pose_index = 0;
             pose_index < poses.size();
             ++pose_index)
        {
            PoseCandidate current = poses[pose_index];
            for (int refinement_index = 0;
                 refinement_index < 2;
                 ++refinement_index)
            {
                std::vector<int> full_subset;
                full_subset.reserve(current.assignments.size());
                for (size_t assignment_index = 0;
                     assignment_index < current.assignments.size();
                     ++assignment_index)
                {
                    full_subset.push_back(
                        static_cast<int>(assignment_index));
                }

                cv::Vec3d solved_orientation;
                cv::Vec3d solved_position;
                if (!call_seeded_iterative_pnp(
                        model_points,
                        observed_points,
                        current.assignments,
                        full_subset,
                        camera_matrix,
                        distortion_coefficients,
                        current.orientation_rvec,
                        current.position_cm,
                        solved_orientation,
                        solved_position,
                        diagnostics))
                {
                    break;
                }

                PoseCandidate refined;
                if (!evaluate_pose(
                        model_points,
                        observed_points,
                        camera_matrix,
                        distortion_coefficients,
                        prior,
                        settings,
                        continuing,
                        focal_length_px,
                        solved_orientation,
                        solved_position,
                        refined))
                {
                    break;
                }

                const bool mapping_changed =
                    refined.assignments != current.assignments;
                current = refined;
                if (!mapping_changed)
                {
                    break;
                }
            }
            refined_poses.push_back(current);
        }

        poses.swap(refined_poses);
        retain_pose_clusters(poses);
        diagnostics.retained_pose_candidate_count =
            static_cast<int>(poses.size());
    }

    static bool validate_inputs(
        const std::vector<cv::Point3f> &model_points_cm,
        const std::vector<cv::Point2f> &image_points_px,
        const cv::Matx33d &camera_matrix,
        const cv::Vec<double, 5> &distortion_coefficients,
        const HMDPointCloudPosePrior &prior,
        const HMDPointCloudPoseSolverSettings &settings)
    {
        if (!prior.has_orientation ||
            !is_finite(prior.orientation_rvec) ||
            (prior.has_position &&
             !is_finite(prior.position_cm)) ||
            model_points_cm.size() < 4 ||
            model_points_cm.size() >
                HMDPointCloudPoseSolver::kMaxModelPointCount ||
            image_points_px.size() < 4 ||
            image_points_px.size() >
                HMDPointCloudPoseSolver::kMaxImagePointCount ||
            settings.min_inlier_count < 4 ||
            settings.min_inlier_count >
                static_cast<int>(model_points_cm.size()) ||
            settings.min_continuation_inlier_count < 4 ||
            settings.min_continuation_inlier_count >
                settings.min_inlier_count ||
            settings.min_association_distance_px <= 0. ||
            settings.max_association_distance_px <
                settings.min_association_distance_px ||
            settings.min_strict_gate_px <= 0. ||
            settings.max_strict_gate_px <
                settings.min_strict_gate_px ||
            settings.max_reprojection_error_px <= 0. ||
            settings.min_depth_cm <= 0. ||
            settings.max_depth_cm <= settings.min_depth_cm ||
            settings.max_lateral_position_cm <= 0. ||
            settings.min_model_pair_distance_cm <= 0. ||
            settings.min_ray_angle_degrees <= 0. ||
            settings.min_blob_pair_distance_px <= 0. ||
            settings.max_pair_translation_residual_cm <= 0. ||
            settings.coarse_geometry_tolerance_cm <= 0. ||
            settings.strict_geometry_tolerance_cm <= 0. ||
            settings.max_normalized_rms_error <= 0. ||
            settings.min_projection_area_px_sqr <= 0. ||
            settings.min_acquisition_coverage <= 0. ||
            settings.min_acquisition_coverage > 1. ||
            settings.min_continuation_coverage <= 0. ||
            settings.min_continuation_coverage > 1. ||
            settings.max_orientation_error_degrees <= 0. ||
            settings.max_acquisition_tilt_error_degrees <= 0. ||
            settings.max_acquisition_heading_error_degrees <= 0. ||
            settings.max_continuation_tilt_error_degrees <= 0. ||
            settings.max_continuation_heading_error_degrees <= 0. ||
            settings.max_translation_jump_cm <= 0. ||
            settings.base_translation_gate_cm <= 0. ||
            settings.translation_gate_speed_cm_per_second < 0. ||
            settings.max_position_prior_age_seconds < 0. ||
            settings.acquisition_yaw_step_degrees <= 0. ||
            settings.acquisition_ambiguity_score_margin < 0. ||
            settings.continuation_ambiguity_score_margin < 0. ||
            !std::isfinite(camera_matrix(0, 0)) ||
            !std::isfinite(camera_matrix(1, 1)) ||
            std::fabs(camera_matrix(0, 0)) <= 1e-6 ||
            std::fabs(camera_matrix(1, 1)) <= 1e-6)
        {
            return false;
        }

        for (size_t model_index = 0;
             model_index < model_points_cm.size();
             ++model_index)
        {
            if (!is_finite(model_points_cm[model_index]))
            {
                return false;
            }
        }
        for (size_t image_index = 0;
             image_index < image_points_px.size();
             ++image_index)
        {
            if (!is_finite(image_points_px[image_index]))
            {
                return false;
            }
        }
        for (int row = 0; row < 3; ++row)
        {
            for (int column = 0; column < 3; ++column)
            {
                if (!std::isfinite(camera_matrix(row, column)))
                {
                    return false;
                }
            }
        }
        for (int coefficient_index = 0;
             coefficient_index < 5;
             ++coefficient_index)
        {
            if (!std::isfinite(
                    distortion_coefficients[coefficient_index]))
            {
                return false;
            }
        }

        return true;
    }
}

bool HMDPointCloudPoseSolver::solve(
    const std::vector<cv::Point3f> &model_points_cm,
    const std::vector<cv::Point2f> &image_points_px,
    const cv::Matx33d &camera_matrix,
    const cv::Vec<double, 5> &distortion_coefficients,
    const HMDPointCloudPosePrior &prior,
    const HMDPointCloudPoseSolverSettings &settings,
    HMDPointCloudPoseResult &result)
{
    result = HMDPointCloudPoseResult();

    if (!validate_inputs(
            model_points_cm,
            image_points_px,
            camera_matrix,
            distortion_coefficients,
            prior,
            settings))
    {
        return false;
    }

    const bool continuing =
        prior.has_position &&
        prior.position_age_seconds >= 0. &&
        prior.position_age_seconds <=
            settings.max_position_prior_age_seconds;
    const double focal_length_px =
        std::sqrt(
            std::fabs(camera_matrix(0, 0)) *
            std::fabs(camera_matrix(1, 1)));

    std::vector<ObservedPoint> observed_points;
    if (!prepare_observed_points(
            image_points_px,
            camera_matrix,
            distortion_coefficients,
            observed_points))
    {
        return false;
    }

    std::vector<OrientationHypothesis> orientations;
    if (!build_orientation_hypotheses(
            prior,
            settings,
            continuing,
            orientations))
    {
        return false;
    }
    result.diagnostics.orientation_hypothesis_count =
        static_cast<int>(orientations.size());

    std::vector<TranslationCandidate> candidates;
    generate_translation_candidates(
        model_points_cm,
        observed_points,
        orientations,
        prior,
        settings,
        focal_length_px,
        candidates,
        result.diagnostics);
    retain_coarse_candidates(
        candidates,
        result.diagnostics);
    if (candidates.empty())
    {
        return false;
    }

    std::vector<MappingHypothesis> mappings;
    build_mapping_hypotheses(
        model_points_cm,
        observed_points,
        orientations,
        candidates,
        focal_length_px,
        settings,
        mappings,
        result.diagnostics);
    if (mappings.empty())
    {
        return false;
    }

    std::vector<PoseCandidate> poses;
    generate_pose_candidates(
        model_points_cm,
        observed_points,
        orientations,
        mappings,
        camera_matrix,
        distortion_coefficients,
        prior,
        settings,
        continuing,
        focal_length_px,
        poses,
        result.diagnostics);
    if (poses.empty())
    {
        return false;
    }

    refine_pose_candidates(
        model_points_cm,
        observed_points,
        camera_matrix,
        distortion_coefficients,
        prior,
        settings,
        continuing,
        focal_length_px,
        poses,
        result.diagnostics);
    if (poses.empty())
    {
        return false;
    }

    std::stable_sort(
        poses.begin(),
        poses.end(),
        pose_less);
    if (poses.size() > 1 &&
        poses[0].match_count == poses[1].match_count)
    {
        const double ambiguity_margin =
            continuing
                ? settings.
                      continuation_ambiguity_score_margin
                : settings.
                      acquisition_ambiguity_score_margin;
        if (poses[0].score - poses[1].score <
            ambiguity_margin)
        {
            result.ambiguous = true;
            return false;
        }
    }

    const PoseCandidate &best_pose = poses[0];
    result.orientation_rvec =
        best_pose.orientation_rvec;
    result.position_cm =
        best_pose.position_cm;
    result.reprojection_error_px =
        best_pose.reprojection_error_px;
    result.projection_area_px_sqr =
        best_pose.projection_area_px_sqr;
    result.projection_coverage =
        best_pose.projection_coverage;
    result.score = best_pose.score;
    result.inlier_pairs.reserve(
        best_pose.assignments.size());
    for (size_t assignment_index = 0;
         assignment_index < best_pose.assignments.size();
         ++assignment_index)
    {
        const std::pair<int, int> &assignment =
            best_pose.assignments[assignment_index];
        result.inlier_pairs.push_back(
            std::make_pair(
                assignment.first,
                observed_points[assignment.second].
                    original_index));
    }

    return true;
}
