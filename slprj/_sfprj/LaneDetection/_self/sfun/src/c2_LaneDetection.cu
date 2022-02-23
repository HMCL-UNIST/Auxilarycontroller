/* Include files */

#include "LaneDetection_sfun.h"
#include "c2_LaneDetection.h"
#include "MWCudaDimUtility.hpp"
#ifdef utFree
#undef utFree
#endif

#ifdef utMalloc
#undef utMalloc
#endif

#ifdef __cplusplus

extern "C" void *utMalloc(size_t size);
extern "C" void utFree(void*);

#else

extern void *utMalloc(size_t size);
extern void utFree(void*);

#endif

#include <cstdlib>
#include <cstring>

/* Type Definitions */

/* Named Constants */
const int32_T CALL_EVENT = -1;

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;

/* Function Declarations */
static void initialize_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static void initialize_params_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct *
  chartInstance);
static void enable_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static void disable_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static void c2_do_animation_call_c2_LaneDetection
  (SFc2_LaneDetectionInstanceStruct *chartInstance);
static void ext_mode_exec_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c2_LaneDetection
  (SFc2_LaneDetectionInstanceStruct *chartInstance);
static void set_sim_state_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c2_st);
static void sf_gateway_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_start_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_terminate_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_setup_runtime_resources_c2_LaneDetection
  (SFc2_LaneDetectionInstanceStruct *chartInstance);
static void mdl_cleanup_runtime_resources_c2_LaneDetection
  (SFc2_LaneDetectionInstanceStruct *chartInstance);
static void initSimStructsc2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static void c2_eML_blk_kernel(SFc2_LaneDetectionInstanceStruct *chartInstance,
  real_T c2_b_in_1[154587], real32_T c2_b_out_1[6]);
static void c2_DeepLearningNetwork_setup(SFc2_LaneDetectionInstanceStruct
  *chartInstance, c2_trainedLaneNet0_LaneDetection0 *c2_obj);
static void c2_DeepLearningNetwork_predict(SFc2_LaneDetectionInstanceStruct
  *chartInstance, c2_trainedLaneNet0_LaneDetection0 *c2_obj, real_T
  c2_varargin_1[154587], real32_T c2_varargout_1[6]);
static void c2_emlrt_marshallIn(SFc2_LaneDetectionInstanceStruct *chartInstance,
  const mxArray *c2_b_out_1, const char_T *c2_identifier, real32_T c2_y[6]);
static void c2_b_emlrt_marshallIn(SFc2_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real32_T c2_y[6]);
static uint8_T c2_c_emlrt_marshallIn(SFc2_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_LaneDetection, const char_T
  *c2_identifier);
static uint8_T c2_d_emlrt_marshallIn(SFc2_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId);
static __global__ void c2_eML_blk_kernel_kernel1(const real_T c2_b_in_1[154587],
  real_T c2_b_r_f1[154587]);
static __global__ void c2_DeepLearningNetwork_predict_kernel2(const real_T
  c2_varargin_1[154587], real32_T c2_in[154587], real_T c2_b_r_f1[154587]);
static __global__ void c2_DeepLearningNetwork_predict_kernel3(const real32_T
  c2_in[154587], c2_cell_wrap_8 c2_miniBatchT[1]);
static __global__ void c2_DeepLearningNetwork_predict_kernel4(const
  c2_cell_wrap_10 c2_outputsMiniBatch[1], real32_T c2_varargout_1[6]);
static void c2_checkCleanupCudaError(cudaError_t c2_errCode, const char_T
  *c2_file, uint32_T c2_line);
static emlrtRTEInfo c2_createEmlrtInfoStruct(const char_T *c2_file, uint32_T
  c2_line);
static void init_dsm_address_info(SFc2_LaneDetectionInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc2_LaneDetectionInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
  emlrtLicenseCheckR2012b(chartInstance->c2_fEmlrtCtx,
    "distrib_computing_toolbox", 2);
  emlrtLicenseCheckR2012b(chartInstance->c2_fEmlrtCtx, "neural_network_toolbox",
    2);
  sim_mode_is_external(chartInstance->S);
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c2_network_not_empty = false;
  chartInstance->c2_is_active_c2_LaneDetection = 0U;
  cudaGetLastError();
  cudaMalloc(&chartInstance->c2_gpu_r_f1, 1236696UL);
  cudaMalloc(&chartInstance->c2_gpu_in_1, 1236696UL);
}

static void initialize_params_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct *
  chartInstance)
{
}

static void enable_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c2_do_animation_call_c2_LaneDetection
  (SFc2_LaneDetectionInstanceStruct *chartInstance)
{
  sfDoAnimationWrapper(chartInstance->S, false, true);
  sfDoAnimationWrapper(chartInstance->S, false, false);
}

static void ext_mode_exec_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c2_LaneDetection
  (SFc2_LaneDetectionInstanceStruct *chartInstance)
{
  const mxArray *c2_b_y = NULL;
  const mxArray *c2_c_y = NULL;
  const mxArray *c2_st;
  const mxArray *c2_y = NULL;
  c2_st = NULL;
  c2_y = NULL;
  sf_mex_assign(&c2_y, sf_mex_createcellmatrix(2, 1), false);
  c2_b_y = NULL;
  sf_mex_assign(&c2_b_y, sf_mex_create("y", *chartInstance->c2_out_1, 1, 0U, 1U,
    0U, 2, 1, 6), false);
  sf_mex_setcell(c2_y, 0, c2_b_y);
  c2_c_y = NULL;
  sf_mex_assign(&c2_c_y, sf_mex_create("y",
    &chartInstance->c2_is_active_c2_LaneDetection, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c2_y, 1, c2_c_y);
  sf_mex_assign(&c2_st, c2_y, false);
  return c2_st;
}

static void set_sim_state_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c2_st)
{
  const mxArray *c2_u;
  c2_u = sf_mex_dup(c2_st);
  c2_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 0)),
                      "out_1", *chartInstance->c2_out_1);
  chartInstance->c2_is_active_c2_LaneDetection = c2_c_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c2_u, 1)),
     "is_active_c2_LaneDetection");
  sf_mex_destroy(&c2_u);
  sf_mex_destroy(&c2_st);
}

static void sf_gateway_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
  int32_T c2_i;
  int32_T c2_i1;
  real32_T c2_fv[6];
  chartInstance->c2_JITTransitionAnimation[0] = 0U;
  _sfTime_ = sf_get_time(chartInstance->S);
  for (c2_i = 0; c2_i < 154587; c2_i++) {
    chartInstance->c2_dv[c2_i] = (*chartInstance->c2_in_1)[c2_i];
  }

  c2_eML_blk_kernel(chartInstance, chartInstance->c2_dv, c2_fv);
  for (c2_i1 = 0; c2_i1 < 6; c2_i1++) {
    (*chartInstance->c2_out_1)[c2_i1] = c2_fv[c2_i1];
  }

  c2_do_animation_call_c2_LaneDetection(chartInstance);
}

static void mdl_start_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static void mdl_terminate_c2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
  cudaError_t c2_errCode;
  cudaFree(*chartInstance->c2_gpu_in_1);
  cudaFree(*chartInstance->c2_gpu_r_f1);
  c2_errCode = cudaGetLastError();
  if (c2_errCode != cudaSuccess) {
    emlrtThinCUDAError(c2_errCode, cudaGetErrorName(c2_errCode),
                       cudaGetErrorString(c2_errCode), "SimGPUErrorChecks",
                       chartInstance->c2_fEmlrtCtx);
  }
}

static void mdl_setup_runtime_resources_c2_LaneDetection
  (SFc2_LaneDetectionInstanceStruct *chartInstance)
{
  setLegacyDebuggerFlag(chartInstance->S, false);
  setDebuggerFlag(chartInstance->S, false);
  sim_mode_is_external(chartInstance->S);
}

static void mdl_cleanup_runtime_resources_c2_LaneDetection
  (SFc2_LaneDetectionInstanceStruct *chartInstance)
{
}

static void initSimStructsc2_LaneDetection(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
}

const mxArray *sf_c2_LaneDetection_get_eml_resolved_functions_info()
{
  const mxArray *c2_nameCaptureInfo = NULL;
  const char_T *c2_data[42] = {
    "789ced9dc993e3585ec7dd133d41374c55273d0cd34cb054079ca6a1d2763ad3652e60cb76a69dde97f45231d129cbcfb66c5992b578bb606e73830882d3b013"
    "cc30c1303411400cdbbf00072efc1d9cb81081bcbc4c5b6595dc29f9952cfd14d1ed56fc94effbfbbdf7fa7df456f9de4b65dff3f97ccfb57ffeeb777cbee9f3",
    "aff896d733dffa3adbfc7ec5b77be9edef6d7eabba7b7c7dd5f7fecedf2dedffa1e9fddee69e1178054d95f50d4f0fd1c35fb68521cbd3bc529989c8272159e0"
    "c6a8bdb274580e55d8212a6fdfe49677c3e496e9e166695afe37d543cca0ac0e7d524f7ef490dbbe59e5c7f2fa6f8378df3f303f4606f971a6b3bf4ed513df39",
    "5765e99c13189a3bcf462b9968ecbc14f407fdad734510b896303d977bb484dae76d8e11da48fabc45cb68f7e6b3d57f9f7fc66ab929f15a3a9c40b7e3088919"
    "444b3ccb7773489908d2e0a5a88bf3bb16e3fcd4244e6c1fd20a47b728cdcb2ee2735aa1ab3cdde25041124424292cc26580fd5a58f4eb174cfcc2f655bebddc",
    "93533a7f3e78cf9a3fdf37f107db5fa7a8ef9cf784213aef0d19ee3c2e30ea10f18a8c2b4659154541520a3433a0bb483ea4a2c8eb3f11f19fac9ff9bccd7dae"
    "d0521769696f6acf6fefc98673834a84f3e5de20ee8f0ecc17fdefe3f31fac7e7ff38bfff97cf94b4a0f5f5ed19b1aa47768bdfe7903bd339d7d30e98e2a2a95",
    "6f86a36cb17417a877e862fffad18f82898e991f3e837b52e97b8517f8fea9713ed3ddebe3c476ad5ddab8906ff57dc005e0027e1eb840420fb8604ffac085c3"
    "e2fc9a499cd8aeb54b9555eb94615bdbfa0b8bfac005e0c253f4f0e5153de0823de97b850b56ebcbcf9ac489ed0f5c88d30abd1a32dcf56361d10fe003f0e129",
    "7af8f28a1ef0c19ef4bdc207abfd869f368913dbb57629955fce36c8bbfa0b8bfac005e0c253f4f0e5153de0823de97b850b56ebcb374de2c47666b926202948"
    "65345211cf20dc2a031f800febe7810f24f4800ff6a4ef153e58ed3718e5eb99eef7b1dfa041a294cb0117800bebe7810b24f4800bf6a40f5c382cce67ba7b7d",
    "9cd83ed1dc48f1a2aa44a5aeec83f124e0027e1eb840420fb8604ffa5ee1c2bdc5383fd0dd3fc6b9b62c5724d1332401078003ebe7810324f48003f6a4ef150e"
    "58ed1f7c89f9e682d668e5d4e1aefec2a23e7001b8f0143d7c79450fb8604ffac085c3e2fc199338b17d22d162965ead52ddd15f58d4072e00179ea2872fafe8",
    "0117ec49df2b5c58588cf35b267162fb98e6d836ade0f5474941a2385a96d9cecc5e7f8013c089a7e8e1cb2b7ac0097bd2f70a27acf61f0e9d77d6daa5151872"
    "f410c1bc3370e1f179e002093de0823de97b850b0b8b711eda7fd8395f2f234c905411ca8aa4328abdfe002780134fd1c39757f48013f6a40f9c382cce43db65",
    "5142a216c36ae96a999da3e5b6060a8d69ee1df9039c004e6cebe1cb2b7ac0097bd2f70a27ac8e333dd7ddebe3c476f9e1fcd5dcea5076e0027061fd3c708184"
    "1e70c19ef4bdc2857b8b717ed530ceb545e3812afa8003c001fc3c7080841e70c09ef4bdc201abfd830f4de2c4762d36aebc61c2b6fec2a23e7001b8f0143d7c",
    "79450fb8604ffac085c3e23c747fc3860bcba9851dfd85457de00270e1297af8f28a1e70c19ef4810b87c579e87cc2920b25246b64506805e613800b0fcf0317"
    "48e80117ec491fb870589c1fe9eef57162fb920b5571b9f36d0d06e0027061fd3c7081841e70c19ef4810b87c579e8773e1589e6655190d11d2d6deb2f2cea03",
    "17800b4fd1c39757f4800bf6a40f5c382c4ea37c3dd3fd8a481aaaca7affc2f21025e0027061fd3c7081841e70c19ef4bdc20552df7fde7021af2a1b30e8fc58"
    "58f403f8007c788a1ebebca2077cb0277de0c361717edd244e6c376a91810fc087f5f3c007127ac0077bd2f70a1fac8e2b1dfabd06696b6dd2b6fec2a2bedd5c",
    "f059e4c29f98f883edaf53798781e1b18448f2e0b3fffb147870443d523cb899371a8352add9baab0493749fe9cbad5222063c38351e58ad2fdf308913db45cd"
    "7f9651a27c7b7b759273b97066910b3f34f107db9dc785bd2575f0fe78bbdab56f03278eaa478a13179162b03d977ba58b6433c167661d3552c9dd00274e8d13",
    "f716e334fbde27b3f93a83537960b59ff04726fe60bbf378804b665d2548b557bf0eedff51f548b5ff09617a554cde95fc287cddcfb5ae6e02f3bb38f4134eae"
    "fdb73a6e74e87e679a51d831adb0022fefe82f2cea3b8d077f6ae20fb63b8f075b25a4d512183772871e291e8cab8956393c11a874466d33296ea63486039847",
    "38391edc5b8cf3a70ce35c5b36a310ae6dffbf67e20fb63baffddf94cca64640fbef0e3d52edffa87ccd15a609aa5d2f75a522f26785496a9c84f6ffd4daff85"
    "c5380ffd0ecf723f7361dde22cbfaca0ca8a30ccd03324c9f6fae3b4f9831f9bf883edcee3837189919c67fed1bffe2ebdfcf54afb4d5a8f142fa49b6c5189b5",
    "23a1b490998da297dd71aa9381fec2c9f182d4be85add109adf529e5723a3f1616fd70dabad4bf32f107db9dc789374a6a5567607ec11d7aa4f840a519d4e558"
    "ae1863eaf141215eed64ba13985f003ee89edbcf070af8b0f9753a1f28c27c80f1a6e3ea91e2432e34938b65a5ded76a756538be9964834208f870727cc0f74f",
    "8df399ee5e1f27b68b0f2317ebbe835bb9f0e726fe60bbf3b8b05342c4d7a502178eab476a3d7aaf94572bb76caac04ba1b2c4472f14e5d5f67a74e0c2fef49d"
    "c60552f56539aa1ddd33b6e4543e7877ffc2de92024eb84c8f54ff61960d312c7f5757042e7631ce8fbbf1493d16074e788d13878e2fedce7ec2fc8373c797de",
    "2829185f72911eb1f125ba2796e6ec759a9d52d3795216af95ac40011f4e8d0fdfb518e7af99c489eda2202b054960902cafcfd6ab0825a4a8124fd10aea0a12"
    "ab05bfedd7c2a25f76f3e2138bbcf889893fd8ee3c5e1c52722f4552ed1d9cab745c3d52fcc8dfdce5b243ba1a8b64e2b97e4bbc0d8e6f8404f0e3d4f861757e",
    "e2b9ee5e1f27b6ef8e6e38971356fb157f61e20fb63b8f13bb25842b0aac6b72871ea9f1e6ab1a9b8f72f98beaa023cc1baad86814a4940fb8e0352e7c99ef44"
    "6f463376f41716f59db66fee74f74d6f9510ec9b768d1ea97e42b550aaf3af92f9ca24dfece7e45781a4a2c03e08eff1e0439338b15d464a94677a82a4d35f58",
    "d4771a0ffed8c41f6c771e0f1e4a08f63db8488fd8be0726f56ad6e6aadd346a0d4637aaac566659d847ed391e1cfa5d68adb5d9b85066e748762c0fac8e1bfd"
    "a5893fd8ee481e6c9710e1f335800bc7d523c58530d795fd177252c9656b4ce32a4ecb7c5c84f54a27c78585c5385f98c489ed222dc9ebaf7fca94e66a17f15b",
    "6b274bb99c533961751efa1f4cfcc176e771c2acc4e09c5637e991e246a538575a593a3aa61297c3d860ca14b37904e34bc00ddd73d8fed65688026e9c183728"
    "d2dc807989e3ea91e286bfa34a9713b1d7bdc9a562a9627a781b913858ffea396efca2499cd8fe662bb49913dd1cc5e1546e58dd4ff785893fd87e0adc782c31",
    "72eb5d8117c7d523c58bab6ca5f8aa91cc4c674cf1b23da4fbf4a4cac1791cc00bdd73a6bc28012f96d709f1a2449217302e755c3d52bcb88d72d7e549ec4ab9"
    "f477520d29cbd5d4781bfa1727c70babf5e5639338b15d46caaaed59cea16e6fc0762a27acce777fdfc41f6c771e27de2c2992e783c3f7458fab478a0f415aa9",
    "84a3a1f9558e0a0b037f91998d7234f4274e8e0fc4f65faf77f06eb73cfb8e75722c2f3cbcfffa809283fe855bf448f1434d2bf9e9dd24a35e67432554eaf9fd"
    "696e0ceba54e8e1f0b8b71be308913dbf7b4423b8708b9951b273cef6d5262b05eca4d7aa4f667c79a699652aad4154d27e85c73ee0fa790ea036e9c1a37f0fd",
    "53e3fc9a499cd8be1a15df6a9a9dca099f454efc99893fd8ee404e6c9510cc6bbb458f543f42ec8c6bd36825324c8c423c23471281c4280afbf14e8e07a4de1f"
    "b4e668db1d4ae03b6cd7e75c2e78f73cf1bd2505e789bb4c8fd8fa2731122f646f7bea9055c3a96cbe1f52729d1be0c4a9716261314ebbdbe77b8bfebcafbb7f",
    "f4676d61b49cdcd623376fed301c18540e52ed169cfb7a5c3d521c184cbaa38a4ae59be1285b2cdd05ea1dbad877d17ebb7b83bf37cb47fd65948f5ed3fbfd27"
    "eae1f4bf6da287edafabdbcd6d34ce08bc2209dcb922d12c8fda199a5f8e91bc1cd2caae7ff706fa76fd7ffa871f7ea34552ef27fff9ad8f49eae1eb54dbbd43",
    "fbbd9368abdee8c52eabf3b89f2ba7949b4b79c4f9dcd3eebdb0f85ef46f06e99fe9ecaf538913782ffa942da12e2b6befd6712422be8d788645f2767e2d0cf2"
    "e3d4dea3f5d7a1ed3abc47c37bb41bf5e03dda9ef481275f8e2756ebdd2726f985ed6d6e13c44b466df3fcfadf51917d571cf9df27eae1f4ffc0440fdb0970a4",
    "2baa6beb764d79c86fadd22cb3fa1c6738e173f27e0bc6e18faa478a1b281c88c572632632ce14fb810225767ae1b68bd67dfa2c72e37b06e99fe9ecce9baf13"
    "1fced8dfce8f8541bcd0cf78bb1ef433de9e2fd0cf78b77ad0cfb0277de0c52e2f48f523f6785455584eb68b5bbf6ce207b61b726be5cdbb38d7c259d564950d",
    "fb60b232905b2ffa1bd0ff38aa1eb175404cba5c676a513fbabef3cfe560a117aad02e5a2f0a3cd9e5896810efa1f9f14c77afcf0f6cd7dcabf2ec4845397a88"
    "7cf6f57fece68867cfbd588364a79848f64760ffd971f548f123309ac767d36aa312eca000938a755a17919a8bcebd007eecf283dcf91848d4bcbd438c224879",
    "5529d34391d39db5b0edd7c2a25f7673c5ea7cd9bf9bf883ed8eab376bae1c527c44bf3707fd95e3ea915ab7d5949866586da24eab1855b8862ae5546efbfd12"
    "78b33f7daff2e6574df203db370d566aa879fe16dcb8963787afcf7058bdd9e1cddb8a8fecf91ac09be3ea91e24d30c707072d6ed48d50955e251f4e73d745c1",
    "07bc712b6facd6ab9f33c90f6cdf3458a55c8e421ca73559a2aa6cfbb1b0e887d3e65bfed6c41f6c775c3dd9e1cb4e71c1f7b25da4476abcacacd0c3095365aa"
    "09741549c8492a5b62eab0decbb53c5918c47b687efc92497e603b6ea0844996eeafce935b8fbdac8e9bb3d11fbbb962f53cc07f32f107db1d575f76b9b2b7d8",
    "48cec7407fe5b87aa4f852f7cf6e3ac157d4057d954a178383db51a65483f97ce08b417e1cfcfda3371aaad5580bc68b6bf9f28f26fe60bbe3ea8b115f1e8b0d"
    "e65f5ca4478a2f915e81a76f932a47052e0ab356377b99aef109e00bf0657f7ebc30c90f6cdf345494c0a9437ecfbbb053f96275dee59f4dfcc176c7d5971dbe",
    "18161bf1f3098133c7d523c51926dfa9cf851093b87835180d8bc140ad7581a01f039c31c88f5f31c90f6cdfdb606d75659cca19cf7e6fe92d9cd9eeca005fdc"
    "a1476a5eff466932a97884bf50c4f83c981cd7c699f4f63839f0657ffaa7ca1762e7efaf1baa2ccbb32d5a617af84b7e4ee58ad579fdbf33f107db1d574f76b8",
    "a22b2eadee004fdca147eaff7b454c258361868d34a471598d05db7361c6b8e83c75e0c92e4facae4bfed4243fb07dd340ad571c197d24d6e75cbe581d1ffb17"
    "137fb0dd71f566872fc6c58707c88037eed023c59bcc6d5f0d5c16d25c9deb172b7781083f569ac01bd7f2666110ef91d6913d34583b9f28b5d11fa78d8fb963",
    "1dd9fe62837564eed123353e369c5d56a846feca9f6bcd731d25c94c2e85a40ff8e256be58ad575f37c90f6cd7dca354591186197a86b4664a109dbdafd2eaf8"
    "d88f4cfcc176c7d59387f361f4c505fb28dda4478a2769fa6a708ba840f826a9a4eb85909409ddc23e4af7f2443488d7ee73c6981e6206a94e52e0daa8ed732e",
    "473c7eced84e31417fc43d7aa4c6bb1a7cb8d96d0d59a5c9ca93cc607a492b013fec9b742d3f087eff7c3d685266e7cb034356efb93ee772c46a7fe4c726fe60"
    "bbe3eac9437fe4cde2029eb8478f144ff2e1d9751451e8b25d49dc6683a162ee424dc0fc09f0c4203f3e36c90f6cd7dc4be557e3245b30f1b997277f63e20fb6",
    "3bae9e3cf044575c84bfe3053c39ae1e299ef452d37c94effb4789793e1d52c23237990d61bf0af0c4203fbe69921fd8aeb957d67c47ed547ed53ca578cd1924"
    "bb96275f98f883ed8eab270f3cd9575cb03fc52d7aa47822df06d8ae7c939e0642856ce142991667bd319cabef5a9e909a2fd1dcc30d139afa9cdb2ff1f87cc9",
    "4e31c1f8967bf4488d6b8f2a857ab044a56f7aa94477986327bc586cf8801fc08ffdf96194ff67ba5fcdbde566b9d872b35c8aef084ee587d57ec80f4dfcc176"
    "c7d58f077eec14139cc3e2323d62dfa74f0fae9aaa1a0d8c832d569e678bedcba404e35aaee5c8c2205ebbf7992c074ad048453c833288ef2abdcdd1ea705ef1",
    "fa725c7d791cdf322c36e8a7b8478f145f6eeffab54a7c181bcef348494855898a746614f0c5ad7cb1da4ff94877afcf0f6c7f5828b4d9b9e8549e581de7fa6b"
    "137fb0dd71f543b79e0bef54047eb8438fd438d79c56b84a4d8a46aed5d14856d857e50915f3013fdcca0fe2eb8293885654092dd79b523ee7f64b605df09bc5",
    "05fd11f7e891ea8f500dd92fddc5aa57d4b4d5e855b9d27540ccc2ba60d7f284d4f7ecf735507b8e57712c5f5e58ac3727fe3dfb438a0fbea7e2223d52bc29c5"
    "93fee6f855713eec0bd73334c8e4a26202bea7e25ade105c37bcb3516ed35239962fb06e785f71c178985bf4488d5b94cacdd02c569936a861949b1503dd70bb",
    "b77d2e06f0647ffa5ee5c9a1dfab5f1fe0c13fb6500eefaf78fc7bf5fb8a0bbe57ef223d523c69466ec68dbb71375965b9dab879cb4bd954c9073c712b4f4483"
    "780fcd8fe7ba7b7d7e60fb6e03e55c8e589d9fff81893fd8eeb8fab18f23f05d14d7e8911adf9adc06fad566bc12a6e7d94e2d5fcb052e151aceed722d3f48bd",
    "978c698e6dd30ac2ab519dbe6ed8e3f3f37b8b0be6e7dda3478a27e5017d379f0af169ad36ed5ee4b2897848ad004f5ccb13abfd11a3fc3fd3fde2066ab5eb3a"
    "2a75ddca9113dfd7a82f26e2fb1a615cebb87aa438721798777b683abca85dc703bc1abb0dcbf5fa3570c4ad1cb15aaf3e31c90f6c5f0d9844f9764d732a2948",
    "3734d759bdeb3a952756f733febd893fd8eeb87ab235beb5a7b860bfbccbf4488d4bb4e6c2acd093b946fba62ede368b17b95211e6dfddcb958541bcefaa3dbf"
    "7fa23ffacbc81f7c915fb7e5aceab0c6879101d66db9458f547fe48a4997eb4c2dea47d777feb91c2cf44215da05e7acfc3f9e707ec9",
    "" };

  c2_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c2_data[0], 134800U, &c2_nameCaptureInfo);
  return c2_nameCaptureInfo;
}

static void c2_eML_blk_kernel(SFc2_LaneDetectionInstanceStruct *chartInstance,
  real_T c2_b_in_1[154587], real32_T c2_b_out_1[6])
{
  cudaMemcpy(chartInstance->c2_gpu_in_1, &c2_b_in_1[0], 1236696UL,
             cudaMemcpyHostToDevice);
  c2_eML_blk_kernel_kernel1<<<dim3(302U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c2_gpu_in_1, *chartInstance->c2_gpu_r_f1);
  if (!chartInstance->c2_network_not_empty) {
    c2_DeepLearningNetwork_setup(chartInstance, &chartInstance->c2_network);
    chartInstance->c2_network_not_empty = true;
  }

  cudaMemcpy(&chartInstance->c2_r_f1[0], chartInstance->c2_gpu_r_f1, 1236696UL,
             cudaMemcpyDeviceToHost);
  c2_DeepLearningNetwork_predict(chartInstance, &chartInstance->c2_network,
    chartInstance->c2_r_f1, c2_b_out_1);
}

static void c2_DeepLearningNetwork_setup(SFc2_LaneDetectionInstanceStruct
  *chartInstance, c2_trainedLaneNet0_LaneDetection0 *c2_obj)
{
  c2_obj->setup();
}

static void c2_DeepLearningNetwork_predict(SFc2_LaneDetectionInstanceStruct
  *chartInstance, c2_trainedLaneNet0_LaneDetection0 *c2_obj, real_T
  c2_varargin_1[154587], real32_T c2_varargout_1[6])
{
  c2_cell_wrap_10 (*c2_gpu_outputsMiniBatch)[1];
  c2_cell_wrap_8 (*c2_gpu_miniBatchT)[1];
  real_T (*c2_b_gpu_r_f1)[154587];
  real_T (*c2_gpu_varargin_1)[154587];
  real32_T (*c2_gpu_in)[154587];
  real32_T (*c2_gpu_varargout_1)[6];
  cudaMalloc(&c2_b_gpu_r_f1, 1236696UL);
  cudaMalloc(&c2_gpu_varargout_1, 24UL);
  cudaMalloc(&c2_gpu_outputsMiniBatch, 24UL);
  cudaMalloc(&c2_gpu_miniBatchT, 618348UL);
  cudaMalloc(&c2_gpu_in, 618348UL);
  cudaMalloc(&c2_gpu_varargin_1, 1236696UL);
  cudaMemcpy(c2_gpu_varargin_1, &c2_varargin_1[0], 1236696UL,
             cudaMemcpyHostToDevice);
  c2_DeepLearningNetwork_predict_kernel2<<<dim3(302U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*c2_gpu_varargin_1, *c2_gpu_in, *c2_b_gpu_r_f1);
  c2_DeepLearningNetwork_predict_kernel3<<<dim3(302U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*c2_gpu_in, *c2_gpu_miniBatchT);
  cudaMemcpy(c2_obj->getInputDataPointer(0), (*c2_gpu_miniBatchT)[0].f1,
             c2_obj->layers[0]->getOutputTensor(0)->getNumElements() * sizeof
             (real32_T), cudaMemcpyDeviceToDevice);
  c2_obj->predict();
  cudaMemcpy((*c2_gpu_outputsMiniBatch)[0].f1, c2_obj->getLayerOutput(17, 0),
             c2_obj->layers[17]->getOutputTensor(0)->getNumElements() * sizeof
             (real32_T), cudaMemcpyDeviceToDevice);
  c2_DeepLearningNetwork_predict_kernel4<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*c2_gpu_outputsMiniBatch, *c2_gpu_varargout_1);
  cudaMemcpy(&c2_varargout_1[0], c2_gpu_varargout_1, 24UL,
             cudaMemcpyDeviceToHost);
  cudaFree(*c2_gpu_varargin_1);
  cudaFree(*c2_gpu_in);
  cudaFree(*c2_gpu_miniBatchT);
  cudaFree(*c2_gpu_outputsMiniBatch);
  cudaFree(*c2_gpu_varargout_1);
  cudaFree(*c2_b_gpu_r_f1);
}

static void c2_emlrt_marshallIn(SFc2_LaneDetectionInstanceStruct *chartInstance,
  const mxArray *c2_b_out_1, const char_T *c2_identifier, real32_T c2_y[6])
{
  emlrtMsgIdentifier c2_thisId;
  c2_thisId.fIdentifier = const_cast<const char_T *>(c2_identifier);
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c2_b_out_1), &c2_thisId, c2_y);
  sf_mex_destroy(&c2_b_out_1);
}

static void c2_b_emlrt_marshallIn(SFc2_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId,
  real32_T c2_y[6])
{
  int32_T c2_i;
  real32_T c2_fv[6];
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), c2_fv, 0, 1, 0U, 1, 0U, 2, 1, 6);
  for (c2_i = 0; c2_i < 6; c2_i++) {
    c2_y[c2_i] = c2_fv[c2_i];
  }

  sf_mex_destroy(&c2_u);
}

static uint8_T c2_c_emlrt_marshallIn(SFc2_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c2_b_is_active_c2_LaneDetection, const char_T
  *c2_identifier)
{
  emlrtMsgIdentifier c2_thisId;
  uint8_T c2_y;
  c2_thisId.fIdentifier = const_cast<const char_T *>(c2_identifier);
  c2_thisId.fParent = NULL;
  c2_thisId.bParentIsCell = false;
  c2_y = c2_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c2_b_is_active_c2_LaneDetection), &c2_thisId);
  sf_mex_destroy(&c2_b_is_active_c2_LaneDetection);
  return c2_y;
}

static uint8_T c2_d_emlrt_marshallIn(SFc2_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c2_u, const emlrtMsgIdentifier *c2_parentId)
{
  uint8_T c2_b_u;
  uint8_T c2_y;
  sf_mex_import(c2_parentId, sf_mex_dup(c2_u), &c2_b_u, 1, 3, 0U, 0, 0U, 0);
  c2_y = c2_b_u;
  sf_mex_destroy(&c2_u);
  return c2_y;
}

static __global__ __launch_bounds__(512, 1) void c2_eML_blk_kernel_kernel1(const
  real_T c2_b_in_1[154587], real_T c2_b_r_f1[154587])
{
  int32_T c2_i;
  c2_i = (int32_T)mwGetGlobalThreadIndex();
  if (c2_i < 154587) {
    c2_b_r_f1[c2_i] = c2_b_in_1[c2_i];
  }
}

static __global__ __launch_bounds__(512, 1) void
  c2_DeepLearningNetwork_predict_kernel2(const real_T c2_varargin_1[154587],
  real32_T c2_in[154587], real_T c2_b_r_f1[154587])
{
  int32_T c2_i;
  c2_i = (int32_T)mwGetGlobalThreadIndex();
  if (c2_i < 154587) {
    c2_b_r_f1[c2_i] = c2_varargin_1[c2_i];
    c2_in[c2_i] = (real32_T)c2_b_r_f1[c2_i];
  }
}

static __global__ __launch_bounds__(512, 1) void
  c2_DeepLearningNetwork_predict_kernel3(const real32_T c2_in[154587],
  c2_cell_wrap_8 c2_miniBatchT[1])
{
  uint64_T c2_threadId;
  uint64_T c2_tmpIndex;
  int32_T c2_i1;
  int32_T c2_i2;
  int32_T c2_p;
  c2_threadId = mwGetGlobalThreadIndex();
  c2_i1 = (int32_T)(c2_threadId % 227UL);
  c2_tmpIndex = (c2_threadId - (uint64_T)c2_i1) / 227UL;
  c2_i2 = (int32_T)(c2_tmpIndex % 227UL);
  c2_tmpIndex = (c2_tmpIndex - (uint64_T)c2_i2) / 227UL;
  c2_p = (int32_T)c2_tmpIndex;
  if ((c2_p < 3) && (c2_i2 < 227) && (c2_i1 < 227)) {
    c2_miniBatchT[0].f1[(c2_i1 + 227 * c2_i2) + 51529 * c2_p] = c2_in[(c2_i2 +
      227 * c2_i1) + 51529 * c2_p];
  }
}

static __global__ __launch_bounds__(32, 1) void
  c2_DeepLearningNetwork_predict_kernel4(const c2_cell_wrap_10
  c2_outputsMiniBatch[1], real32_T c2_varargout_1[6])
{
  int32_T c2_i3;
  c2_i3 = (int32_T)mwGetGlobalThreadIndex();
  if (c2_i3 < 6) {
    c2_varargout_1[c2_i3] = c2_outputsMiniBatch[0].f1[c2_i3];
  }
}

void c2_trainedLaneNet0_LaneDetection0::allocate()
{
  int32_T c2_idx;
  this->targetImpl->allocate(290400, 2);
  for (c2_idx = 0; c2_idx < 18; c2_idx++) {
    this->layers[c2_idx]->allocate();
  }

  (static_cast<MWTensor<real32_T> *>(this->inputTensors[0]))->setData
    (this->layers[0]->getLayerOutput(0));
}

void c2_trainedLaneNet0_LaneDetection0::postsetup()
{
  this->targetImpl->postSetup(this->layers, this->numLayers);
}

c2_trainedLaneNet0_LaneDetection0::c2_trainedLaneNet0_LaneDetection0()
{
  this->numLayers = 18;
  this->isInitialized = false;
  this->targetImpl = 0;
  this->layers[0] = new MWInputLayer;
  this->layers[0]->setName("data");
  this->layers[1] = new MWElementwiseAffineLayer;
  this->layers[1]->setName("data_normalization");
  this->layers[1]->setInPlaceIndex(0, 0);
  this->layers[2] = new MWFusedConvReLULayer;
  this->layers[2]->setName("conv1_relu1");
  this->layers[3] = new MWNormLayer;
  this->layers[3]->setName("norm1");
  this->layers[4] = new MWMaxPoolingLayer;
  this->layers[4]->setName("pool1");
  this->layers[5] = new MWFusedConvReLULayer;
  this->layers[5]->setName("conv2_relu2");
  this->layers[6] = new MWNormLayer;
  this->layers[6]->setName("norm2");
  this->layers[7] = new MWMaxPoolingLayer;
  this->layers[7]->setName("pool2");
  this->layers[8] = new MWFusedConvReLULayer;
  this->layers[8]->setName("conv3_relu3");
  this->layers[9] = new MWFusedConvReLULayer;
  this->layers[9]->setName("conv4_relu4");
  this->layers[10] = new MWFusedConvReLULayer;
  this->layers[10]->setName("conv5_relu5");
  this->layers[11] = new MWMaxPoolingLayer;
  this->layers[11]->setName("pool5");
  this->layers[12] = new MWFCLayer;
  this->layers[12]->setName("fc6");
  this->layers[13] = new MWReLULayer;
  this->layers[13]->setName("relu6");
  this->layers[13]->setInPlaceIndex(0, 0);
  this->layers[14] = new MWFCLayer;
  this->layers[14]->setName("fcLane1");
  this->layers[15] = new MWReLULayer;
  this->layers[15]->setName("fcLane1Relu");
  this->layers[15]->setInPlaceIndex(0, 0);
  this->layers[16] = new MWFCLayer;
  this->layers[16]->setName("fcLane2");
  this->layers[17] = new MWOutputLayer;
  this->layers[17]->setName("output");
  this->layers[17]->setInPlaceIndex(0, 0);
  this->targetImpl = new MWTargetNetworkImpl;
  this->inputTensors[0] = new MWTensor<real32_T>;
  this->inputTensors[0]->setHeight(227);
  this->inputTensors[0]->setWidth(227);
  this->inputTensors[0]->setChannels(3);
  this->inputTensors[0]->setBatchSize(1);
  this->inputTensors[0]->setSequenceLength(1);
}

void c2_trainedLaneNet0_LaneDetection0::deallocate()
{
  int32_T c2_idx;
  this->targetImpl->deallocate();
  for (c2_idx = 0; c2_idx < 18; c2_idx++) {
    this->layers[c2_idx]->deallocate();
  }
}

void c2_trainedLaneNet0_LaneDetection0::setSize()
{
  int32_T c2_idx;
  for (c2_idx = 0; c2_idx < 18; c2_idx++) {
    this->layers[c2_idx]->propagateSize();
  }

  this->allocate();
  this->postsetup();
}

void c2_trainedLaneNet0_LaneDetection0::resetState()
{
}

void c2_trainedLaneNet0_LaneDetection0::setup()
{
  if (this->isInitialized) {
    this->resetState();
  } else {
    this->isInitialized = true;
    this->targetImpl->preSetup();
    this->targetImpl->setAutoTune(true);
    (static_cast<MWInputLayer *>(this->layers[0]))->createInputLayer
      (this->targetImpl, this->inputTensors[0], 227, 227, 3, 0, "", 0);
    (static_cast<MWElementwiseAffineLayer *>(this->layers[1]))
      ->createElementwiseAffineLayer(this->targetImpl, this->layers[0]
      ->getOutputTensor(0), 227, 227, 3, 227, 227, 3, false, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_data_scale.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_data_offset.bin",
      0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[2]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[1]
      ->getOutputTensor(0), 11, 11, 3, 96, 4, 4, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv1_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv1_b.bin",
      1);
    (static_cast<MWNormLayer *>(this->layers[3]))->createNormLayer
      (this->targetImpl, this->layers[2]->getOutputTensor(0), 5, 0.0001, 0.75,
       1.0, 0);
    (static_cast<MWMaxPoolingLayer *>(this->layers[4]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[3]->getOutputTensor(0), 3, 3, 2, 2, 0, 0,
       0, 0, 0, 1, 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[5]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[4]
      ->getOutputTensor(0), 5, 5, 48, 128, 1, 1, 2, 2, 2, 2, 1, 1, 2,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv2_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv2_b.bin",
      0);
    (static_cast<MWNormLayer *>(this->layers[6]))->createNormLayer
      (this->targetImpl, this->layers[5]->getOutputTensor(0), 5, 0.0001, 0.75,
       1.0, 1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[7]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[6]->getOutputTensor(0), 3, 3, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[8]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[7]
      ->getOutputTensor(0), 3, 3, 256, 384, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv3_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv3_b.bin",
      1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[9]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[8]
      ->getOutputTensor(0), 3, 3, 192, 192, 1, 1, 1, 1, 1, 1, 1, 1, 2,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv4_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv4_b.bin",
      0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[10]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[9]
      ->getOutputTensor(0), 3, 3, 192, 128, 1, 1, 1, 1, 1, 1, 1, 1, 2,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv5_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_conv5_b.bin",
      1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[11]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[10]->getOutputTensor(0), 3, 3, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFCLayer *>(this->layers[12]))->createFCLayer(this->targetImpl,
      this->layers[11]->getOutputTensor(0), 9216, 4096,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_fc6_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_fc6_b.bin",
      1);
    (static_cast<MWReLULayer *>(this->layers[13]))->createReLULayer
      (this->targetImpl, this->layers[12]->getOutputTensor(0), 1);
    (static_cast<MWFCLayer *>(this->layers[14]))->createFCLayer(this->targetImpl,
      this->layers[13]->getOutputTensor(0), 4096, 16,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_fcLane1_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_fcLane1_b.bin",
      0);
    (static_cast<MWReLULayer *>(this->layers[15]))->createReLULayer
      (this->targetImpl, this->layers[14]->getOutputTensor(0), 0);
    (static_cast<MWFCLayer *>(this->layers[16]))->createFCLayer(this->targetImpl,
      this->layers[15]->getOutputTensor(0), 16, 6,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_fcLane2_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_trainedLaneNet0_LaneDetection0_fcLane2_b.bin",
      1);
    (static_cast<MWOutputLayer *>(this->layers[17]))->createOutputLayer
      (this->targetImpl, this->layers[16]->getOutputTensor(0), 1);
    this->outputTensors[0] = this->layers[17]->getOutputTensor(0);
    this->setSize();
  }
}

void c2_trainedLaneNet0_LaneDetection0::predict()
{
  int32_T c2_idx;
  for (c2_idx = 0; c2_idx < 18; c2_idx++) {
    this->layers[c2_idx]->predict();
  }
}

void c2_trainedLaneNet0_LaneDetection0::cleanup()
{
  int32_T c2_idx;
  this->deallocate();
  for (c2_idx = 0; c2_idx < 18; c2_idx++) {
    this->layers[c2_idx]->cleanup();
  }

  if (this->targetImpl) {
    this->targetImpl->cleanup();
  }
}

real32_T *c2_trainedLaneNet0_LaneDetection0::getLayerOutput(int32_T
  c2_layerIndex, int32_T c2_portIndex)
{
  return this->layers[c2_layerIndex]->getLayerOutput(c2_portIndex);
}

real32_T *c2_trainedLaneNet0_LaneDetection0::getInputDataPointer(int32_T
  c2_index)
{
  return (static_cast<MWTensor<real32_T> *>(this->inputTensors[c2_index]))
    ->getData();
}

real32_T *c2_trainedLaneNet0_LaneDetection0::getInputDataPointer()
{
  return (static_cast<MWTensor<real32_T> *>(this->inputTensors[0]))->getData();
}

real32_T *c2_trainedLaneNet0_LaneDetection0::getOutputDataPointer(int32_T
  c2_index)
{
  return (static_cast<MWTensor<real32_T> *>(this->outputTensors[c2_index]))
    ->getData();
}

real32_T *c2_trainedLaneNet0_LaneDetection0::getOutputDataPointer()
{
  return (static_cast<MWTensor<real32_T> *>(this->outputTensors[0]))->getData();
}

int32_T c2_trainedLaneNet0_LaneDetection0::getBatchSize()
{
  return this->inputTensors[0]->getBatchSize();
}

c2_trainedLaneNet0_LaneDetection0::~c2_trainedLaneNet0_LaneDetection0()
{
  int32_T c2_idx;
  this->cleanup();
  c2_checkCleanupCudaError(cudaGetLastError(), __FILE__, __LINE__);
  for (c2_idx = 0; c2_idx < 18; c2_idx++) {
    delete this->layers[c2_idx];
  }

  if (this->targetImpl) {
    delete this->targetImpl;
  }

  delete this->inputTensors[0];
}

static void c2_checkCleanupCudaError(cudaError_t c2_errCode, const char_T
  *c2_file, uint32_T c2_line)
{
  emlrtRTEInfo c2_rtInfo;
  if ((c2_errCode != cudaSuccess) && (c2_errCode != cudaErrorCudartUnloading)) {
    c2_rtInfo = c2_createEmlrtInfoStruct(c2_file, c2_line);
    emlrtCUDAWarning(c2_errCode, cudaGetErrorName(c2_errCode),
                     cudaGetErrorString(c2_errCode), &c2_rtInfo);
  }
}

static emlrtRTEInfo c2_createEmlrtInfoStruct(const char_T *c2_file, uint32_T
  c2_line)
{
  emlrtRTEInfo c2_b_rtInfo;
  uint32_T c2_len;
  char_T *c2_brk;
  char_T *c2_fn;
  char_T *c2_pn;
  c2_len = (uint32_T)strlen(c2_file);
  c2_pn = (char_T *)calloc(c2_len + 1U, 1U);
  c2_fn = (char_T *)calloc(c2_len + 1U, 1U);
  memcpy(c2_pn, c2_file, c2_len);
  memcpy(c2_fn, c2_file, c2_len);
  c2_brk = strrchr(c2_fn, '.');
  *c2_brk = '\x00';
  c2_brk = NULL;
  c2_brk = strrchr(c2_fn, '/');
  if (c2_brk == NULL) {
    c2_brk = strrchr(c2_fn, '\\');
  }

  if (c2_brk == NULL) {
    c2_brk = c2_fn;
  } else {
    c2_brk++;
  }

  c2_b_rtInfo.lineNo = c2_line;
  c2_b_rtInfo.colNo = 0;
  c2_b_rtInfo.fName = c2_brk;
  c2_b_rtInfo.pName = c2_pn;
  return c2_b_rtInfo;
}

static void init_dsm_address_info(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static void init_simulink_io_address(SFc2_LaneDetectionInstanceStruct
  *chartInstance)
{
  chartInstance->c2_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c2_in_1 = (real_T (*)[154587])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c2_out_1 = (real32_T (*)[6])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c2_LaneDetection_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2645480056U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3584261826U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(751965951U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2291345985U);
}

mxArray *sf_c2_LaneDetection_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,1);
  mxSetCell(mxcell3p, 0, mxCreateString("dltargets.cudnn.cudnnApi"));
  return(mxcell3p);
}

mxArray *sf_c2_LaneDetection_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("GPUAcceleration");
  mxArray *hiddenFallbackType = mxCreateString("late");
  mxArray *hiddenFallbackReason = mxCreateString("ir_function_calls");
  mxArray *incompatibleSymbol = mxCreateString("#__setup__");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c2_LaneDetection_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = sf_mex_decode(
    "eNpjYPT0ZQACPiDm4GRgYAPRQMzIAAGsSHxOJHGQ+h8M+NUzoal3QFLPgkU9H5J6ASg/MSUlOL+"
    "0KDnVLTMntRgi1kDAXkY0ewMI2CuDZi+I7xvumpOam5pXUp5ZnOqYlpaZl+qTWJlapJdcUAAz98"
    "MA+b+CRP97ELBXAs1eCbD/3UqLU1Oc8/PKglJ9QuF+B5n3YoD8XUCivwnZy41mL4ifnJcXn1iQC"
    "YvmYRXPgmj2CoLj2dnPDxy7nrkFOXrJpcMwf8uj2SuPJ3/DQgFkLkjTQPi/g0T/RxCwVxXNXlUC"
    "/vdOLcpLhaaF4ZQOpNHslcZRziHnhOGU/8XQ7BUD+z8ksSg9tcQvtaQ8vygb7vXhFO+SaPZKQsq"
    "90uKS/FxwhLvlFzmXuvj5wUt9BgYA8ZCiqg=="
    );
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c2_LaneDetection(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNpjYPT0ZQACPhDBxMDABqQ4IEwwYIXyGaFijHBxFri4AhCXVBakgsSLi5I9U4B0XmIumJ9YWuG"
    "Zl5YPNt+CAWE+GxbzGZHM54SKQ8AHe8r0izig62fBop8VSb8AlJ9fWhJvCAsfKE2+OxQcKNMPsT"
    "+AgD+k0PwB4mcWxycml2SWpcYnG8X7JOaluqSWpAIF8vMQ5oIAAB9fGm0="
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c2_LaneDetection_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "soqOeZDjTPcCrKLUGh7SS3D";
}

static void sf_opaque_initialize_c2_LaneDetection(void *chartInstanceVar)
{
  initialize_params_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
    chartInstanceVar);
  initialize_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c2_LaneDetection(void *chartInstanceVar)
{
  enable_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c2_LaneDetection(void *chartInstanceVar)
{
  disable_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c2_LaneDetection(void *chartInstanceVar)
{
  sf_gateway_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c2_LaneDetection(SimStruct* S)
{
  return get_sim_state_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c2_LaneDetection(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_cleanup_runtime_resources_c2_LaneDetection(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc2_LaneDetectionInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_LaneDetection_optimization_info();
    }

    mdl_cleanup_runtime_resources_c2_LaneDetection
      ((SFc2_LaneDetectionInstanceStruct*) chartInstanceVar);
    ((SFc2_LaneDetectionInstanceStruct*) chartInstanceVar)->
      ~SFc2_LaneDetectionInstanceStruct();
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c2_LaneDetection(void *chartInstanceVar)
{
  mdl_start_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_mdl_terminate_c2_LaneDetection(void *chartInstanceVar)
{
  mdl_terminate_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c2_LaneDetection(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  sf_warn_if_symbolic_dimension_param_changed(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
      sf_get_chart_instance_ptr(S));
    initSimStructsc2_LaneDetection((SFc2_LaneDetectionInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

const char* sf_c2_LaneDetection_get_post_codegen_info(void)
{
  int i;
  const char* encStrCodegen [22] = {
    "eNrtV81v40QUd6ruipVgVSEkpBUfe0FwQKQfy8cJ2uYDItJthdNF4hKm4+d4NuMZd2actH8JV44",
    "cOfBH8Cdw4oLEGXHmwBvbTVPHTpqNSBO0kVx37N9783uf8+xUWkcO/h7iFb3jOPfx/gpeG076u5",
    "etK2NX+nzTeT9b/4JCIg5PiCKhdqb+BAnhG9CSx4ZJ0RK+LIQx4YMCQREbSWXKtGkWxpyJfjMW1",
    "OrT3waMBm4gY+4doizxjgW/RG1RbE5QT50poKYJ4JlAybgXNDnpjRgrM6wFQPs6DqeZoMG4cWRp",
    "6aOYGxZxaFwAbQltCDLW19xcQwzUzEWpmdZS7V4BZRhxRkShtQHRLkToYAOnkYd/j2ODRuVhNCD",
    "KHEJABqDbrJ/olALyOpnGF2dMECMVI7wR8poVnOR2wpHPkfSAT3EIcjtUQPqRZMKUx99toqUNQc",
    "441OEs7pVrc+E8tsF/xmAIqtRvfk0OQJEeHIvSTROHNC6SaI2yZBJmWAjPiDqgGD8NXmn2YuZol",
    "2CcoIMSZTBIjGzpjmIDdG+ptjhs2cycVTJxmAZbz4Il2hoDmBaFkbYmFTXCuS6FdWTUhgHwRGud",
    "GDIdlmotxmnNvI5EB9v0Lq+GWDAMfAarSeGxwnANcoCk7zzFxnITSWNtZFjD5K2325OvJ2EtYUD",
    "5hEJRF1CEaUCfJe4t1+YxbWOPQGRlEnpF4DRDZqEc7ceiPpSqjz6Z0kSuTbARLQWGuoexxEo41V",
    "g002A2lrNwlNAAPNtgGIcjLBvEFvhE29Z2gHU3YOayDpoqFhVENcaqwzbUsAl1GcGp6As5FE0lQ",
    "zfr8WkUsDNgDw8xBp2kxgRFVUwbbBfsensPAJOSKMFE7xDbnLpsIsnCiNlzb9u5PvdevcW5dyWX",
    "v38wpqdSoMcZu+f3fbAxfd8N/K+Sye2Pyb2W22czJ2dxW3j98Gv4x4eEf99675+3//y79vsi+/+",
    "8Md+c8DBbv3XVkEcFNpjIa4v9aozXZoH+N8f0b2VrLc+P4bv6884Jramv26dfBp+67l49jc8Mvh",
    "s5vlfPH9uTAbMxyWNFW142wNg1idNj3er/bIzv/Rn+eJA9T39/fbGY/Bv7efkif93L+cuuZWy6O",
    "7n8fXEej/cXk0/3P5lhx6OcHY+SOaBLbHeBLt3ttonA2cJAcsxP1ue8+f5S7qXcKstVXvD8Wrac",
    "s2S5Re2b9xxfNfz2lD7q5PBbK2zHovPVf43/zZlvDno3W38++iSqBYx7BdNx9hpnWL/o7f8kT21",
    "RTsO/nsP/OGM+kLm8tutqIEOoBiHl1YM6fiwYJXlV80g9r3a1b283hgZ8CNyv2g+vKo58VSpE1+",
    "C3nwDP4p6C2b45ZWx3fWof7HaHH50xsQY8z27NM8zxDJfCE5UO9jJvOivOcyfz5qrz3F0Tnjtzx",
    "T3K8YyWwtMjhnQ1JRws1dX255M1ifvemvD8eE360u5cPO/yPNqZ49y8y7ivR34+uRH3n2bwPM/x",
    "PF9e/5S+r8FkXG873/VzfPtLytNPRtFfdZ7DuXjebd2nPv0XWQnEqA==",
    ""
  };

  static char newstr [1557] = "";
  newstr[0] = '\0';
  for (i = 0; i < 22; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c2_LaneDetection(SimStruct *S)
{
  const char* newstr = sf_c2_LaneDetection_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(3815622807U));
  ssSetChecksum1(S,(1617715500U));
  ssSetChecksum2(S,(503063881U));
  ssSetChecksum3(S,(3728995046U));
}

static void mdlRTW_c2_LaneDetection(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c2_LaneDetection(SimStruct *S)
{
  SFc2_LaneDetectionInstanceStruct *chartInstance;
  chartInstance = (SFc2_LaneDetectionInstanceStruct *)utMalloc(sizeof
    (SFc2_LaneDetectionInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc2_LaneDetectionInstanceStruct));
  chartInstance = new (chartInstance) SFc2_LaneDetectionInstanceStruct;
  chartInstance->chartInfo.chartInstance = chartInstance;
  if (ssGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME && ssGetOffsetTime(S, 0) ==
      0 && ssGetNumContStates(ssGetRootSS(S)) > 0) {
    sf_error_out_about_continuous_sample_time_with_persistent_vars(S);
  }

  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c2_LaneDetection;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c2_LaneDetection;
  chartInstance->chartInfo.mdlStart = sf_opaque_mdl_start_c2_LaneDetection;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c2_LaneDetection;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c2_LaneDetection;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c2_LaneDetection;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c2_LaneDetection;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c2_LaneDetection;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c2_LaneDetection;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c2_LaneDetection;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c2_LaneDetection;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c2_LaneDetection;
  chartInstance->chartInfo.callGetHoverDataForMsg = NULL;
  chartInstance->chartInfo.extModeExec = NULL;
  chartInstance->chartInfo.restoreLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.restoreBeforeLastMajorStepConfiguration = NULL;
  chartInstance->chartInfo.storeCurrentConfiguration = NULL;
  chartInstance->chartInfo.callAtomicSubchartUserFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartAutoFcn = NULL;
  chartInstance->chartInfo.callAtomicSubchartEventFcn = NULL;
  chartInstance->chartInfo.chartStateSetterFcn = NULL;
  chartInstance->chartInfo.chartStateGetterFcn = NULL;
  chartInstance->S = S;
  chartInstance->chartInfo.dispatchToExportedFcn = NULL;
  sf_init_ChartRunTimeInfo(S, &(chartInstance->chartInfo), false, 0,
    chartInstance->c2_JITStateAnimation,
    chartInstance->c2_JITTransitionAnimation);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c2_LaneDetection(chartInstance);
}

void c2_LaneDetection_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c2_LaneDetection(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c2_LaneDetection(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c2_LaneDetection(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c2_LaneDetection_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
