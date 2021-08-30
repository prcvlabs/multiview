
#include "training-data-point.hpp"

#include <opencv2/ml.hpp>

#include "perceive/geometry/normalize-data.hpp"
#include "perceive/geometry/skeleton/bone.hpp"
#include "perceive/geometry/skeleton/p2d-affinity.hpp"
#include "perceive/io/json-io.hpp"
#include "perceive/utils/memstream.hpp"

#define This TrainingDataPoint

namespace perceive::calibration
{
// ------------------------------------------------------------------ operator==

static bool report_eq(const TrainingDataPoint& u,
                      const TrainingDataPoint& v,
                      std::stringstream* ss_ptr) noexcept
{
   bool is_equal = true;

   auto f_is_same = [&](const float a, const float b) -> bool {
      return float_is_same(a, b);
   };

   auto test_vec = [&](const auto& a, const auto& b, auto&& f) -> bool {
      Expects(a.size() == b.size());
      for(decltype(a.size()) i = 0u; i < a.size(); ++i)
         if(!f(a[i], b[i])) return false;
      return true;
   };

   auto test = [&](const auto& a, const auto& b) -> bool {
      if constexpr(std::is_same_v<const int&, decltype(a)>) return a == b;
      if constexpr(std::is_same_v<const float&, decltype(a)>)
         return float_is_same(a, b);
      if constexpr(std::is_same_v<const PoseAnnotation&, decltype(a)>)
         return (a == b);
      if constexpr(std::is_same_v<const bool&, decltype(a)>) return (a == b);
      if constexpr(std::is_same_v<const decltype(This::joints)&, decltype(a)>)
         return test_vec(a, b, f_is_same);
   };

   auto make_str = [&](const auto& a, const auto& b) -> string {
      constexpr bool is_joint_type
          = std::is_same_v<const decltype(This::joints)&, decltype(a)>;
      if constexpr(is_joint_type) {
         bool is_first = true;
         std::stringstream ss{""};
         for(const auto& [x, y] : views::zip(a, b)) {
            if(is_first) {
               is_first = false;
            } else {
               ss << ",\n";
            }
            ss << format("({:g} == {:g})", x, y);
         }
         return indent(ss.str(), 3);
      } else {
         return format("({} == {})", str(a), str(b));
      }
   };

   auto test_eq = [&](const auto& a, const auto& b, const string_view name) {
      const bool eq = test(a, b);
      if(ss_ptr) {
         (*ss_ptr) << format(
             "{:25s}:   {:5s} <- {}\n", name, str(eq), make_str(a, b));
      }
      if(!eq) is_equal = false;
   };

#define TEST(x) test_eq(u.x, v.x, #x)
   TEST(frame_no);
   TEST(speed);
   TEST(bg_delta_score);
   TEST(z_theta);
   TEST(joints);
   TEST(pose);
   TEST(is_false_positive);
#undef TEST
   return is_equal;
}

bool This::operator==(const TrainingDataPoint& o) const noexcept
{
   return report_eq(*this, o, nullptr);
}

bool This::operator!=(const TrainingDataPoint& o) const noexcept
{
   return !(*this == o);
}

// ------------------------------------------------------------------- to-string

string This::to_string() const noexcept
{
   Json::StyledWriter writer;
   return writer.write(to_json());
}

string This::to_json_string() const noexcept
{
   std::stringstream ss{""};
   ss << to_json();
   return ss.str();
}

// ---------------------------------------------------------------- to/from json

Json::Value This::to_json() const noexcept
{
   using namespace ::perceive::skeleton;

   const auto& o = *this;

   Json::Value x{Json::objectValue};
   x["frame-no"]          = json_save(o.frame_no);
   x["speed"]             = json_save(o.speed);
   x["bg-delta-score"]    = json_save(o.bg_delta_score);
   x["z-theta"]           = json_save(o.z_theta);
   x["joints"]            = json_save(cbegin(o.joints), cend(o.joints));
   const string pose_s    = str(o.pose);
   x["pose"]              = json_save(pose_s);
   x["is-false-positive"] = json_save(o.is_false_positive);

   if(true) {
      TrainingDataPoint z;
      bool success = false;
      try {
         if(!z.load_json(x)) FATAL(format("failed to load-json"));
         success = (o == z);
      } catch(std::exception& e) {
         FATAL(format("logic error: {}", e.what()));
      }
      if(!success) {
         std::stringstream ss;
         report_eq(o, z, &ss);
         LOG_ERR(format("serialization of TrainingDataPoint failed, report:"));
         cout << ss.str();
         FATAL("logic error");
      }
   }

   return x;
}

static std::pair<bool, This>
load_w_wout_defaults(const Json::Value& x,
                     const TrainingDataPoint* defaults) noexcept
{
   bool success = true;
   This o;
   const string_view op = "reading training-data-point";

   auto try_load
       = [&](const Json::Value& val, auto& dst, const string_view key) {
            auto okay = json_try_load_key(dst, val, key, op, true);
            if(!okay) success = false;
         };

   auto try_load_arr = [&](const Json::Value& node,
                           array<float, TrainingDataPoint::k_n_joints>& dst,
                           const string_view key) {
      auto fail = [&](const string_view msg) {
         success = false;
         WARN(format("{}", msg));
      };
      if(!has_key(node, key)) {
         fail(format("failed to find key '{}'", key));
         return;
      }
      auto o = node[key.data()];
      if(o.type() != Json::arrayValue || o.size() != dst.size()) {
         fail(format("expects array of size {} for key '{}'", dst.size(), key));
         return;
      }
      for(size_t i = 0; i < dst.size(); ++i) json_load(o[int(i)], dst.at(i));
   };

   auto read_enum
       = [&](const Json::Value& x, auto& dst, const string_view key, auto&& F) {
            string s;
            auto okay = json_try_load_key(s, x, key, op, true);
            if(!okay) success = false;
            if(okay) {
               try {
                  dst = F(s);
               } catch(std::exception& e) {
                  //
               }
            }
         };

   try_load(x, o.frame_no, "frame-no");
   try_load(x, o.speed, "speed");
   try_load(x, o.bg_delta_score, "bg-delta-score");
   try_load(x, o.z_theta, "z-theta");
   try_load_arr(x, o.joints, "joints");
   read_enum(x, o.pose, "pose", [](const string_view s) {
      return to_pose_annotation(s);
   });
   try_load(x, o.is_false_positive, "is-false-positive");

   return std::make_pair(success, o);
}

bool This::load_json(const Json::Value& x) noexcept(false)
{
   auto [success, o] = load_w_wout_defaults(x, nullptr);
   if(success) *this = std::move(o);
   return success;
}

void This::read_with_defaults(const Json::Value& x,
                              const TrainingDataPoint* defaults) noexcept
{
   This tmp          = (defaults) ? *defaults : This{};
   auto [success, o] = load_w_wout_defaults(x, &tmp);
   *this             = std::move(o);
}

// --------------------------------------------------------------------- headers
// Total columns = flat_row_size + n_labels
static constexpr int k_n_labels       = 2;
static constexpr int k_n_feature_cols = 4 + This::k_n_joints + 1;

vector<string> This::headers() noexcept // std::bad_alloc
{
   vector<string> o;
   o.resize(k_n_labels + k_n_feature_cols);

   size_t pos = 0;
   auto push  = [&](const string_view x) {
      Expects(pos < o.size());
      o.at(pos++) = string(begin(x), end(x));
   };

   // ---------------------- Headers
   push("is_true_positive");
   push("pose");
   // ---------------------- Samples
   push("frame_no");
   push("speed");
   push("bg_delta_score");
   push("z_theta");
   for(auto i = 0; i < This::k_n_joints; ++i)
      push(format("{:02d}_{}", i, str(skeleton::int_to_joint_label(i))));
   push("missing_count");

   Expects(pos == k_n_labels + k_n_feature_cols);
   return o;
}

// --------------------------------------------------------------------- flatten

vector<real> This::flatten() const noexcept // std::bad_alloc
{
   vector<real> o;
   o.resize(k_n_feature_cols);

   size_t pos = 0;
   auto push  = [&](const auto x) {
      Expects(pos < o.size());
      o.at(pos++) = real(x);
   };

   // push(frame_no);
   push(fNAN); // Everything works better WITHOUT frame-no
   push(speed);
   push(bg_delta_score);
   push(z_theta);
   for(auto i = 0; i < This::k_n_joints; ++i) push(joints.at(size_t(i)));

   // Now output the total number of missing values into the last column
   auto count_missing = [](const auto start, const auto finish) -> real {
      int counter = 0;
      for(auto ii = start; ii != finish; ++ii)
         if(!std::isfinite(*ii)) ++counter;
      return real(counter);
   };
   push(count_missing(cbegin(o), cbegin(o) + long(pos)));

   Expects(pos == k_n_feature_cols);

   return o;
}

// ---------------------------------------------------------------------- labels

vector<real> This::labels() const noexcept // std::bad_alloc
{
   vector<real> o;
   o.reserve(2);
   o.push_back(int(pose));
   o.push_back(is_false_positive);
   return o;
}

// ----------------------------------------------------- make-training-datapoint

TrainingDataPoint make_training_datapoint(const SceneDescription& scene_desc,
                                          const Skeleton2DInfo& skel_info,
                                          const PoseAnnotation pose,
                                          const real speed,
                                          const bool is_false_positive)
{
   Expects(skel_info.p2d_ptr != nullptr);
   Expects(skel_info.patches.size() > 0);
   const auto& p2d = *skel_info.p2d_ptr;
   const auto& p3d = p2d.best_3d_result();
   const auto& Xs  = p3d.Xs();

   const LABImage lab_still = scene_desc.sensor_LAB_still(p2d.sensor_no());
   const auto patchw        = skel_info.patches[0].width;
   const auto patchh        = skel_info.patches[0].height;
   const auto still_patches
       = p2d.make_image_patches(int(patchw), int(patchh), lab_still);
   const float patch_score
       = calc_lab_patch_score(still_patches, skel_info.patches);

   auto get_bone_vector = [&](const skeleton::Bone& bone) -> Vector3f {
      try {
         return (Xs.at(size_t(bone.kp0)) - Xs.at(size_t(bone.kp1)));
      } catch(std::exception&) {
         // do nothing
      }
      return Vector3f::nan();
   };

   auto calc_theta = [&](const skeleton::Joint& joint) -> float {
      return dot(get_bone_vector(skeleton::get_bone(joint.b0)).normalised(),
                 get_bone_vector(skeleton::get_bone(joint.b1)).normalised());
   };

   TrainingDataPoint o;
   o.frame_no          = p2d.frame_no();
   o.speed             = float(speed);
   o.bg_delta_score    = lab_score_to_lab_p(patch_score);
   o.z_theta           = dot(Vector3f(0.0, 0.0, 1.0f), p3d.up_n().normalised());
   o.pose              = pose;
   o.is_false_positive = is_false_positive;
   o.is_init           = true;
   Expects(o.joints.size() == size_t(skeleton::Joint::k_n_joints));
   for(auto i = 0; i < skeleton::Joint::k_n_joints; ++i)
      o.joints.at(size_t(i))
          = calc_theta(skeleton::get_joint(skeleton::int_to_joint_label(i)));
   return o;
}

// --------------------------------------------------------- make-feature-vector

VectorXr make_feature_vector(const SceneDescription& scene_desc,
                             const Skeleton2DInfo& skel_info,
                             const real speed)
{
   const auto tdp = make_training_datapoint(
       scene_desc, skel_info, PoseAnnotation::NONE, speed, false);

   const auto Xs = tdp.flatten();
   VectorXr o(Xs.size());
   for(size_t i = 0; i < Xs.size(); ++i) o(long(i)) = Xs.at(i);
   return o;
}

// ------------------------------------------------- encode-as-svm-training-data

string encode_as_pose_training_data(const vector<TrainingDataPoint>& o) noexcept
{
   std::stringstream ss{""};

   for(const auto& tdp : o) {
      const auto Xs = tdp.flatten();
      if(!std::any_of(cbegin(Xs), cend(Xs), [](const auto& X) {
            return std::isfinite(X);
         }))
         continue; // the row is empty

      ss << format("{}", int(tdp.pose));
      for(size_t col = 0; col < Xs.size(); ++col) {
         const auto X = Xs.at(col);
         if(std::isfinite(X)) ss << format(" {}:{}", col, X);
      }
      ss << "\n";
   }

   return ss.str();
}

string encode_as_tpfp_training_data(const vector<TrainingDataPoint>& o) noexcept
{
   std::stringstream ss{""};

   for(const auto& tdp : o) {
      const auto Xs = tdp.flatten();
      if(!std::any_of(cbegin(Xs), cend(Xs), [](const auto& X) {
            return std::isfinite(X);
         }))
         continue; // the row is empty

      ss << format("{}", (tdp.is_false_positive ? -1 : 1));
      for(size_t col = 0; col < Xs.size(); ++col) {
         const auto X = Xs.at(col);
         if(std::isfinite(X)) ss << format(" {}:{}", col, X);
      }
      ss << "\n";
   }

   return ss.str();
}

static string cv_mat_32f_row_to_str(const cv::Mat& mat, const int row)
{
   std::stringstream ss{""};
   for(int col = 0; col < mat.cols; ++col) {
      if(col > 0) ss << ", ";
      ss << mat.at<float>(row, col);
   }
   return ss.str();
}

std::pair<cv::Mat, cv::Mat>
collate_training_samples(const vector<TrainingDataPoint>& data,
                         const bool do_impute_missing_values) noexcept
{
   if(data.size() == 0) return {};

   const int N = int(data.size()); // number of rows;

   auto calc_features = [&]() {
      const int cols = int(data.front().flatten().size());
      Expects(cols > 0);

      cv::Mat samples(N, cols, CV_32F);

      int row = 0;
      for(const auto& tdp : data) {
         const auto Xs = tdp.flatten();
         for(int col = 0; col < cols; ++col)
            samples.at<float>(row, col) = float(Xs.at(size_t(col)));
         ++row;
      }

      if(do_impute_missing_values) samples = impute_missing_values(samples);

      return samples;
   };

   auto calc_labels = [&]() {
      cv::Mat labels(N, 2, CV_32S);

      int row = 0;
      for(const auto& tdp : data) {
         labels.at<int32_t>(row, 0) = (tdp.is_false_positive ? -1 : 1);
         labels.at<int32_t>(row, 1) = int(tdp.pose);
         ++row;
      }

      return labels;
   };

   const auto samples = calc_features();
   const auto labels  = calc_labels();

   return {samples, labels};
}

// ------------------------------------------------------------------ encode-csv
// Return '?' for missing values, unless they're imputed
string encode_csv(const vector<TrainingDataPoint>& data,
                  const bool impute_missing_values) noexcept
{
   std::stringstream ss{""};

   const auto [mat, lbls]
       = collate_training_samples(data, impute_missing_values);

   const auto headers = TrainingDataPoint::headers();

   Expects(headers.size() == size_t(mat.cols) + size_t(lbls.cols));
   Expects(lbls.rows == mat.rows);
   {
      bool first = true;
      for(const auto& header : headers) {
         if(first)
            first = false;
         else
            ss << ',';
         ss << '"' << header << '"';
      }
      ss << '\n';
   }

   for(int row = 0; row < mat.rows; ++row) {
      for(int col = 0; col < lbls.cols; ++col) {
         if(col > 0) ss << ',';
         ss << lbls.at<int32_t>(row, col);
      }
      for(int col = 0; col < mat.cols; ++col) {
         ss << ",";
         const float x = mat.at<float>(row, col);
         if(std::isfinite(x))
            ss << x;
         else
            ss << '?';
      }
      ss << "\n";
   }

   return ss.str();
}

// ------------------------------------------------------------------ decode-csv

std::tuple<vector<string>, cv::Mat, vector<float>>
decode_csv(const string_view raw_csv,
           const bool do_impute_missing_values,
           const bool has_header_line) noexcept(false)
{
   vector<string> headers;
   vector<float> raw;

   auto safe_lexical_cast = [](const string& s) -> float {
      char* str_end = nullptr;
      const auto x  = strtof(&s[0], &str_end);
      return (s.size() > 0 && str_end == &s[0] + s.size() && std::isfinite(x))
                 ? x
                 : fNAN;
   };

   auto decode_csv_row = [](const string& line, const char delim) {
      constexpr char escape = '"';
      vector<string> cols;

      enum EscapeStatus : int { ESCAPE_NONE, IN_ESCAPE, ONE_DQUOTE };
      EscapeStatus escape_status = ESCAPE_NONE;

      string field;
      auto finish_field = [&]() {
         cols.push_back(field); // copy
         field.clear();
         escape_status = ESCAPE_NONE;
      };

      for(auto c : line) {
         if(c == delim && escape_status != IN_ESCAPE) {
            finish_field();
         } else if(c == escape) {
            switch(escape_status) {
            case ESCAPE_NONE: escape_status = IN_ESCAPE; break;
            case IN_ESCAPE: escape_status = ONE_DQUOTE; break;
            case ONE_DQUOTE:
               escape_status = IN_ESCAPE;
               field.push_back(c);
               break;
            }
         } else if(escape_status == ONE_DQUOTE) {
            throw std::runtime_error("malformed field: single '\"'");
         } else {
            field.push_back(c);
         }
      }
      finish_field();

      return cols;
   };

   size_t raw_row_size = 0;
   auto check_row_size = [&](const auto& raw_row) {
      if(raw_row_size == 0) {
         raw_row_size = raw_row.size();
         return true;
      }
      return raw_row_size == raw_row.size();
   };

   // Decode the CSV stream
   auto istream = imemstream(raw_csv.data(), raw_csv.size());
   string line;
   int lineno = 0;
   while(istream.good()) {
      ++lineno;
      std::getline(istream, line, '\n');
      if(line.empty() || line.front() == '#') continue;
      const auto raw_row = decode_csv_row(line, ',');
      if(!check_row_size(raw_row)) {
         LOG_ERR(format("{}", line));
         throw std::runtime_error(
             format("line #{}, expected {} columns, but got {}",
                    lineno,
                    headers.size(),
                    raw_row.size()));
      } else if(headers.size() == 0 && has_header_line) {
         headers = raw_row;
      } else {
         std::transform(cbegin(raw_row),
                        cend(raw_row),
                        std::back_inserter(raw),
                        safe_lexical_cast);
      }
   }

   // Copy the data into a cv::Mat
   const int cols = int(headers.size());
   const int rows = int(raw.size()) / cols;
   Expects(size_t(rows) * size_t(cols) == raw.size());

   auto out = cv::Mat(rows, cols, CV_32F);
   {
      size_t read_pos = 0;
      for(int row = 0; row < rows; ++row)
         for(int col = 0; col < cols; ++col)
            out.at<float>(row, col) = raw.at(read_pos++);
      Expects(read_pos == raw.size());
   }

   const auto medians = calc_column_medians(out);

   // Impute as necessary
   return {headers,
           do_impute_missing_values ? impute_missing_values(out) : out,
           medians};
}

// --------------------------------------------------------- calc-column-medians

vector<float> calc_column_medians(const cv::Mat& mat) noexcept
{
   const int N    = mat.rows;
   const int cols = mat.cols;

   vector<float> Xs;
   auto compute_median = [&Xs, N](const cv::Mat& samples, const int col) {
      Xs.clear();
      Xs.reserve(size_t(N));
      for(auto row = 0; row < N; ++row)
         if(std::isfinite(samples.at<float>(row, col)))
            Xs.push_back(samples.at<float>(row, col));

      float median = 0.0f;
      if(Xs.size() > 0) {
         const auto stats = calc_sample_statistics(begin(Xs), end(Xs));
         Expects(std::isfinite(stats.median));
         median = float(stats.median);
      }

      return median;
   };

   vector<float> medians;
   medians.resize(size_t(cols));
   for(int col = 0; col < cols; ++col)
      medians.at(size_t(col)) = compute_median(mat, col);

   return medians;
}

// -------------------------------------------------------- impute-missing-value

cv::Mat impute_missing_values(const cv::Mat& mat) noexcept
{
   Expects(mat.type() == CV_32F);
   cv::Mat out;
   mat.copyTo(out);

   const auto medians = calc_column_medians(mat);

   const int N    = mat.rows;
   const int cols = mat.cols;

   for(int row = 0; row < N; ++row)
      for(int col = 0; col < cols; ++col)
         if(!std::isfinite(out.at<float>(row, col)))
            out.at<float>(row, col) = medians.at(size_t(col));

   return out;
}

// ---------------------------------------------------------------- scale-values
// All columns are multiplied by a scalar so that the values
// are in the range [-1..1]
std::pair<cv::Mat, vector<Vector2f>>
scale_values(const cv::Mat& samples) noexcept
{
   cv::Mat out = samples;

   vector<Vector2f> ranges;
   ranges.resize(size_t(samples.cols));

   const int N = samples.rows;
   vector<float> X;
   auto scale_column = [&](int col) {
      X.clear();
      X.reserve(size_t(N));
      for(auto row = 0; row < N; ++row)
         if(std::isfinite(samples.at<float>(row, col)))
            X.push_back(samples.at<float>(row, col));
      if(X.size() == 0) {
         ranges.at(size_t(col)) = Vector2f(0.0f, 0.0f);
      } else {
         const auto stats       = calc_sample_statistics(begin(X), end(X));
         ranges.at(size_t(col)) = to_vec2f(Vector2(stats.min, stats.max));
      }

      const auto& R     = ranges.at(size_t(col));
      const float range = (R(1) - R(0));
      if(range > 0.0f) {
         for(auto row = 0; row < N; ++row) {
            const float x           = samples.at<float>(row, col);
            out.at<float>(row, col) = (x - R(0)) / range;
         }
      }
   };
   for(int c = 0; c < samples.cols; ++c) scale_column(c);

   return {out, ranges};
}

// ------------------------------------------------------- prepare-training-data
// Returns {samples, labels}
std::pair<cv::Mat, cv::Mat>
prepare_training_data(const cv::Mat& raw,
                      const vector<int>& sample_cols,
                      const int label_col) noexcept
{
   const auto cols = remove_duplicates_copy(sample_cols);

   const int N      = raw.rows;
   const int N_cols = raw.cols;

   auto col_in_range = [&N_cols](int col) { return col >= 0 && col < N_cols; };
   Expects(cols.size() == sample_cols.size());
   Expects(col_in_range(label_col));
   Expects(std::all_of(cbegin(cols), cend(cols), col_in_range));
   Expects(std::find(cbegin(cols), cend(cols), label_col) == cend(cols));

   auto samples = cv::Mat(N, int(cols.size()), CV_32F);
   auto labels  = cv::Mat(N, 1, CV_32S);

   for(auto row = 0; row < N; ++row) {
      for(size_t ind = 0; ind < cols.size(); ++ind)
         samples.at<float>(row, int(ind)) = raw.at<float>(row, cols[ind]);
      labels.at<int32_t>(row, 0) = int32_t(raw.at<float>(row, label_col));
   }

   return {samples, labels};
}

// --------------------------------------------------------- make-validation-set

TrainingData
make_training_validation_set(const cv::Mat& samples,
                             const cv::Mat& labels,
                             const real validation_set_size, // [0..1]
                             const size_t random_seed)
{
   Expects(validation_set_size >= 0.0 && validation_set_size <= 1.0);
   Expects(samples.rows == labels.rows);
   Expects(labels.cols == 1);
   TrainingData out;

   // How many rows/columns do we need in output
   const int n_cols  = samples.cols;
   const int total_N = samples.rows;
   const int valid_N
       = std::min(int(std::round(validation_set_size * total_N)), total_N);
   const int training_N = total_N - valid_N;
   Expects(training_N >= 0 && training_N <= total_N);
   Expects(valid_N >= 0 && valid_N <= total_N);

   // Get a random set of indices, N total elements, choosing R elements
   auto make_random_indices
       = [random_seed](const int N,
                       const int R) -> std::pair<vector<int>, vector<int>> {
      Expects(R <= N && N >= 0 && R >= 0);
      vector<int> o((size_t(N)));
      std::iota(begin(o), end(o), 0); // counting...
      std::shuffle(begin(o), end(o), std::mt19937{random_seed});
      const auto partition_itr = std::next(begin(o), R);
      vector<int> t(begin(o), partition_itr); // training indices
      vector<int> v(partition_itr, end(o));   // validation indices
      std::sort(begin(t), end(t)); // maybe better caching behaviour during
      std::sort(begin(v), end(v)); // copy below
      return {t, v};
   };
   const auto [t_indices, v_indices] = make_random_indices(total_N, training_N);

   // copy out the results
   auto make_it = [&](const vector<int>& inds) -> std::pair<cv::Mat, cv::Mat> {
      auto s        = cv::Mat(int(inds.size()), n_cols, CV_32F);
      auto l        = cv::Mat(int(inds.size()), 1, CV_32S);
      int write_pos = 0;
      for(const auto ind : inds) {
         samples.row(ind).copyTo(s.row(write_pos));
         labels.row(ind).copyTo(l.row(write_pos));
         write_pos++;
      }
      Expects(size_t(write_pos) == inds.size());
      return {s, l};
   };
   std::tie(out.training_samples, out.training_labels)     = make_it(t_indices);
   std::tie(out.validation_samples, out.validation_labels) = make_it(v_indices);

   return out;
}

} // namespace perceive::calibration
