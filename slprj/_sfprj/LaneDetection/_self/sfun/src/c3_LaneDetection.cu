/* Include files */

#include "LaneDetection_sfun.h"
#include "c3_LaneDetection.h"
#include "MWCudaDimUtility.hpp"
#include "mwmathutil.h"
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

/* Type Definitions */

/* Named Constants */
const int32_T CALL_EVENT = -1;

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;
static __device__ real_T c3_gpu_vehicleXPoints[28];
static __device__ real_T c3_gpu_laneCoeffMeans[6];
static __device__ real_T c3_gpu_laneCoeffStds[6];

/* Function Declarations */
static void initialize_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static void initialize_params_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct *
  chartInstance);
static void enable_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static void disable_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static void c3_do_animation_call_c3_LaneDetection
  (SFc3_LaneDetectionInstanceStruct *chartInstance);
static void ext_mode_exec_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c3_LaneDetection
  (SFc3_LaneDetectionInstanceStruct *chartInstance);
static void set_sim_state_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_st);
static void sf_gateway_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_start_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_terminate_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_setup_runtime_resources_c3_LaneDetection
  (SFc3_LaneDetectionInstanceStruct *chartInstance);
static void mdl_cleanup_runtime_resources_c3_LaneDetection
  (SFc3_LaneDetectionInstanceStruct *chartInstance);
static void initSimStructsc3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static void c3_eML_blk_kernel(SFc3_LaneDetectionInstanceStruct *chartInstance,
  real32_T c3_b_laneNetOut[6], boolean_T *c3_b_laneFound, real32_T c3_b_ltPts[56],
  real32_T c3_b_rtPts[56]);
static boolean_T c3_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_laneFound, const char_T *c3_identifier);
static boolean_T c3_b_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static void c3_c_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_ltPts, const char_T *c3_identifier,
  real32_T c3_y[56]);
static void c3_d_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real32_T c3_y[56]);
static void c3_e_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_laneCoeffMeans, const char_T
  *c3_identifier, boolean_T *c3_svPtr, real_T c3_y[6]);
static void c3_f_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T *c3_svPtr, real_T c3_y[6]);
static void c3_g_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_vehicleXPoints, const char_T
  *c3_identifier, boolean_T *c3_svPtr, real_T c3_y[28]);
static void c3_h_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T *c3_svPtr, real_T c3_y[28]);
static uint8_T c3_i_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_LaneDetection, const char_T
  *c3_identifier);
static uint8_T c3_j_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId);
static __global__ void c3_eML_blk_kernel_kernel1(const real_T c3_dv[6]);
static __global__ void c3_eML_blk_kernel_kernel2(const real_T c3_dv1[6]);
static __global__ void c3_eML_blk_kernel_kernel3(const real32_T c3_b_laneNetOut
  [6], real32_T c3_params[6]);
static __global__ void c3_eML_blk_kernel_kernel4();
static __global__ void c3_eML_blk_kernel_kernel5(const real32_T c3_params[6],
  real32_T c3_rt_y[28]);
static __global__ void c3_eML_blk_kernel_kernel6(const real32_T c3_params,
  real32_T c3_rt_y[28]);
static __global__ void c3_eML_blk_kernel_kernel7(const real32_T c3_params[6],
  real32_T c3_lt_y[28]);
static __global__ void c3_eML_blk_kernel_kernel8(const real32_T c3_params,
  real32_T c3_lt_y[28]);
static __global__ void c3_eML_blk_kernel_kernel9(const real_T c3_Tinv[9], real_T
  c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel10(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel11(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel12(const real_T c3_t3, const
  real_T c3_t2, const real_T c3_x[9], const int32_T c3_p1, real_T c3_Tinv[9]);
static __global__ void c3_eML_blk_kernel_kernel13(const real_T c3_t3, const
  real_T c3_t2, const real_T c3_x[9], const int32_T c3_p2, real_T c3_Tinv[9]);
static __global__ void c3_eML_blk_kernel_kernel14(const real_T c3_t3, const
  real_T c3_t2, const real_T c3_x[9], const int32_T c3_p3, real_T c3_Tinv[9]);
static __global__ void c3_eML_blk_kernel_kernel15(const real32_T c3_lt_y[28],
  real32_T c3_fv[84]);
static __global__ void c3_eML_blk_kernel_kernel16(const real_T c3_Tinv[9], const
  real32_T c3_fv[84], real32_T c3_U[84]);
static __global__ void c3_eML_blk_kernel_kernel17(real32_T c3_U[56], real32_T
  c3_b_U[84], real32_T c3_b[56]);
static __global__ void c3_eML_blk_kernel_kernel18(const real32_T c3_U[84],
  real32_T c3_b_ltPts[56]);
static __global__ void c3_eML_blk_kernel_kernel19(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel20(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel21(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel22(const real_T c3_t3, const
  real_T c3_t2, const int32_T c3_p3, real_T c3_Tinv[9]);
static __global__ void c3_eML_blk_kernel_kernel23(const real32_T c3_rt_y[28],
  real32_T c3_fv1[84]);
static __global__ void c3_eML_blk_kernel_kernel24(const real_T c3_Tinv[9], const
  real32_T c3_fv1[84], real32_T c3_U[84]);
static __global__ void c3_eML_blk_kernel_kernel25(real32_T c3_U[56], real32_T
  c3_b_U[84], real32_T c3_b[56]);
static __global__ void c3_eML_blk_kernel_kernel26(const real32_T c3_U[84],
  real32_T c3_b_rtPts[56]);
static void init_dsm_address_info(SFc3_LaneDetectionInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc3_LaneDetectionInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
  emlrtLicenseCheckR2012b(chartInstance->c3_fEmlrtCtx,
    "distrib_computing_toolbox", 2);
  emlrtLicenseCheckR2012b(chartInstance->c3_fEmlrtCtx, "image_toolbox", 2);
  sim_mode_is_external(chartInstance->S);
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c3_laneCoeffMeans_not_empty = false;
  chartInstance->c3_laneCoeffStds_not_empty = false;
  chartInstance->c3_vehicleXPoints_not_empty = false;
  chartInstance->c3_is_active_c3_LaneDetection = 0U;
  cudaGetLastError();
  cudaMalloc(&chartInstance->c3_gpu_params, 4UL);
  cudaMalloc(&chartInstance->c3_gpu_fv, 336UL);
  cudaMalloc(&chartInstance->c3_gpu_x, 72UL);
  cudaMalloc(&chartInstance->c3_b_gpu_t3, 8UL);
  cudaMalloc(&chartInstance->c3_b_gpu_params, 4UL);
  cudaMalloc(&chartInstance->c3_gpu_t2, 8UL);
  cudaMalloc(&chartInstance->c3_gpu_Tinv, 72UL);
  cudaMalloc(&chartInstance->c3_gpu_dv, 48UL);
  cudaMalloc(&chartInstance->c3_gpu_ltPts, 224UL);
  cudaMalloc(&chartInstance->c3_gpu_dv1, 48UL);
  cudaMalloc(&chartInstance->c3_c_gpu_U, 336UL);
  cudaMalloc(&chartInstance->c3_gpu_b, 224UL);
  cudaMalloc(&chartInstance->c3_c_gpu_params, 24UL);
  cudaMalloc(&chartInstance->c3_b_gpu_U, 224UL);
  cudaMalloc(&chartInstance->c3_b_gpu_Tinv, 72UL);
  cudaMalloc(&chartInstance->c3_gpu_p3, 4UL);
  cudaMalloc(&chartInstance->c3_gpu_U, 224UL);
  cudaMalloc(&chartInstance->c3_gpu_p1, 4UL);
  cudaMalloc(&chartInstance->c3_gpu_laneNetOut, 24UL);
  cudaMalloc(&chartInstance->c3_gpu_rtPts, 224UL);
  cudaMalloc(&chartInstance->c3_gpu_p2, 4UL);
  cudaMalloc(&chartInstance->c3_gpu_t3, 8UL);
  cudaMalloc(&chartInstance->c3_b_gpu_t2, 8UL);
  cudaMalloc(&chartInstance->c3_gpu_lt_y, 112UL);
  cudaMalloc(&chartInstance->c3_gpu_fv1, 336UL);
  cudaMalloc(&chartInstance->c3_gpu_rt_y, 112UL);
  cudaMalloc(&chartInstance->c3_b_gpu_p3, 4UL);
}

static void initialize_params_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct *
  chartInstance)
{
}

static void enable_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c3_do_animation_call_c3_LaneDetection
  (SFc3_LaneDetectionInstanceStruct *chartInstance)
{
  sfDoAnimationWrapper(chartInstance->S, false, true);
  sfDoAnimationWrapper(chartInstance->S, false, false);
}

static void ext_mode_exec_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c3_LaneDetection
  (SFc3_LaneDetectionInstanceStruct *chartInstance)
{
  const mxArray *c3_b_y = NULL;
  const mxArray *c3_c_y = NULL;
  const mxArray *c3_d_y = NULL;
  const mxArray *c3_e_y = NULL;
  const mxArray *c3_f_y = NULL;
  const mxArray *c3_g_y = NULL;
  const mxArray *c3_h_y = NULL;
  const mxArray *c3_st;
  const mxArray *c3_y = NULL;
  c3_st = NULL;
  c3_y = NULL;
  sf_mex_assign(&c3_y, sf_mex_createcellmatrix(7, 1), false);
  c3_b_y = NULL;
  sf_mex_assign(&c3_b_y, sf_mex_create("y", chartInstance->c3_laneFound, 11, 0U,
    0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 0, c3_b_y);
  c3_c_y = NULL;
  sf_mex_assign(&c3_c_y, sf_mex_create("y", *chartInstance->c3_ltPts, 1, 0U, 1U,
    0U, 2, 28, 2), false);
  sf_mex_setcell(c3_y, 1, c3_c_y);
  c3_d_y = NULL;
  sf_mex_assign(&c3_d_y, sf_mex_create("y", *chartInstance->c3_rtPts, 1, 0U, 1U,
    0U, 2, 28, 2), false);
  sf_mex_setcell(c3_y, 2, c3_d_y);
  c3_e_y = NULL;
  if (!chartInstance->c3_laneCoeffMeans_not_empty) {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_e_y, sf_mex_create("y", chartInstance->c3_laneCoeffMeans,
      0, 0U, 1U, 0U, 2, 1, 6), false);
  }

  sf_mex_setcell(c3_y, 3, c3_e_y);
  c3_f_y = NULL;
  if (!chartInstance->c3_laneCoeffMeans_not_empty) {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_f_y, sf_mex_create("y", chartInstance->c3_laneCoeffStds, 0,
      0U, 1U, 0U, 2, 1, 6), false);
  }

  sf_mex_setcell(c3_y, 4, c3_f_y);
  c3_g_y = NULL;
  if (!chartInstance->c3_vehicleXPoints_not_empty) {
    sf_mex_assign(&c3_g_y, sf_mex_create("y", NULL, 0, 0U, 1U, 0U, 2, 0, 0),
                  false);
  } else {
    sf_mex_assign(&c3_g_y, sf_mex_create("y", chartInstance->c3_vehicleXPoints,
      0, 0U, 1U, 0U, 2, 1, 28), false);
  }

  sf_mex_setcell(c3_y, 5, c3_g_y);
  c3_h_y = NULL;
  sf_mex_assign(&c3_h_y, sf_mex_create("y",
    &chartInstance->c3_is_active_c3_LaneDetection, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c3_y, 6, c3_h_y);
  sf_mex_assign(&c3_st, c3_y, false);
  return c3_st;
}

static void set_sim_state_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_st)
{
  const mxArray *c3_u;
  c3_u = sf_mex_dup(c3_st);
  *chartInstance->c3_laneFound = c3_emlrt_marshallIn(chartInstance, sf_mex_dup
    (sf_mex_getcell(c3_u, 0)), "laneFound");
  c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 1)),
                        "ltPts", *chartInstance->c3_ltPts);
  c3_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 2)),
                        "rtPts", *chartInstance->c3_rtPts);
  c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 3)),
                        "laneCoeffMeans",
                        &chartInstance->c3_laneCoeffMeans_not_empty,
                        chartInstance->c3_laneCoeffMeans);
  c3_e_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 4)),
                        "laneCoeffStds",
                        &chartInstance->c3_laneCoeffStds_not_empty,
                        chartInstance->c3_laneCoeffStds);
  c3_g_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 5)),
                        "vehicleXPoints",
                        &chartInstance->c3_vehicleXPoints_not_empty,
                        chartInstance->c3_vehicleXPoints);
  chartInstance->c3_is_active_c3_LaneDetection = c3_i_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c3_u, 6)),
     "is_active_c3_LaneDetection");
  sf_mex_destroy(&c3_u);
  sf_mex_destroy(&c3_st);
}

static void sf_gateway_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
  int32_T c3_i;
  int32_T c3_i1;
  int32_T c3_i2;
  real32_T c3_fv1[56];
  real32_T c3_fv2[56];
  real32_T c3_fv[6];
  boolean_T c3_b;
  chartInstance->c3_JITTransitionAnimation[0] = 0U;
  _sfTime_ = sf_get_time(chartInstance->S);
  for (c3_i = 0; c3_i < 6; c3_i++) {
    c3_fv[c3_i] = (*chartInstance->c3_laneNetOut)[c3_i];
  }

  c3_eML_blk_kernel(chartInstance, c3_fv, &c3_b, c3_fv1, c3_fv2);
  for (c3_i1 = 0; c3_i1 < 56; c3_i1++) {
    (*chartInstance->c3_rtPts)[c3_i1] = c3_fv2[c3_i1];
  }

  for (c3_i2 = 0; c3_i2 < 56; c3_i2++) {
    (*chartInstance->c3_ltPts)[c3_i2] = c3_fv1[c3_i2];
  }

  *chartInstance->c3_laneFound = c3_b;
  c3_do_animation_call_c3_LaneDetection(chartInstance);
}

static void mdl_start_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static void mdl_terminate_c3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
  cudaError_t c3_errCode;
  cudaFree(*chartInstance->c3_gpu_dv);
  cudaFree(*chartInstance->c3_gpu_U);
  cudaFree(*chartInstance->c3_gpu_Tinv);
  cudaFree(*chartInstance->c3_gpu_dv1);
  cudaFree(*chartInstance->c3_gpu_lt_y);
  cudaFree(*chartInstance->c3_gpu_ltPts);
  cudaFree(chartInstance->c3_gpu_t3);
  cudaFree(*chartInstance->c3_gpu_fv);
  cudaFree(chartInstance->c3_b_gpu_t2);
  cudaFree(chartInstance->c3_b_gpu_t3);
  cudaFree(chartInstance->c3_gpu_p3);
  cudaFree(*chartInstance->c3_gpu_x);
  cudaFree(chartInstance->c3_b_gpu_params);
  cudaFree(chartInstance->c3_gpu_params);
  cudaFree(chartInstance->c3_gpu_t2);
  cudaFree(chartInstance->c3_gpu_p1);
  cudaFree(*chartInstance->c3_gpu_rt_y);
  cudaFree(chartInstance->c3_b_gpu_p3);
  cudaFree(*chartInstance->c3_c_gpu_params);
  cudaFree(*chartInstance->c3_c_gpu_U);
  cudaFree(*chartInstance->c3_b_gpu_Tinv);
  cudaFree(*chartInstance->c3_gpu_rtPts);
  cudaFree(*chartInstance->c3_gpu_fv1);
  cudaFree(chartInstance->c3_gpu_p2);
  cudaFree(*chartInstance->c3_gpu_laneNetOut);
  cudaFree(*chartInstance->c3_gpu_b);
  cudaFree(*chartInstance->c3_b_gpu_U);
  c3_errCode = cudaGetLastError();
  if (c3_errCode != cudaSuccess) {
    emlrtThinCUDAError(c3_errCode, cudaGetErrorName(c3_errCode),
                       cudaGetErrorString(c3_errCode), "SimGPUErrorChecks",
                       chartInstance->c3_fEmlrtCtx);
  }
}

static void mdl_setup_runtime_resources_c3_LaneDetection
  (SFc3_LaneDetectionInstanceStruct *chartInstance)
{
  setLegacyDebuggerFlag(chartInstance->S, false);
  setDebuggerFlag(chartInstance->S, false);
  sim_mode_is_external(chartInstance->S);
}

static void mdl_cleanup_runtime_resources_c3_LaneDetection
  (SFc3_LaneDetectionInstanceStruct *chartInstance)
{
}

static void initSimStructsc3_LaneDetection(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
}

const mxArray *sf_c3_LaneDetection_get_eml_resolved_functions_info()
{
  const mxArray *c3_nameCaptureInfo = NULL;
  const char_T *c3_data[9] = {
    "789ced5a5d6fd25018ee0c331ae336135de22ebcf62319730c36ae0c857543c6c6e4633063a0b487b65bdb036df9f2c65d7aa55e68a2ffc04ba33f40fd27fe07"
    "fd0152a083568fdd2c1ea4f424e4f0e6a1e779fabe3d4fce399498892767088298eb7cde3d23880f61a2dbaef63a62a1df5f20cccd8acff4fb2b96d868b384cf",
    "749d8ebfeef0bdecc70c9435d0d27a814c4be0f44a164a824ccb5aa65d05840254283600db452a8208328204d2c3c1ae1e49d410741ae890fe3dca03e6385d97"
    "088557070ac5e1a09b0fbd9510f7ebb3c987b559f361fd9dc177f2977cc6f84b367c062ed23228b240038c2640b9c840a8b09d346b4035eb2939d47311a9a787",
    "a882cc8960c0f7d621df1a92cf8c3fde7ce2e7a104fcbcc488fe484c7ffe1428fad1795996f4ebecf2327f469dd67ef0fb4bddfee69b1b659c7c5f1e7c5dc3c9"
    "67b471f1b510e39df5395b44f02d58702a9bdc0f54ca91607d23980e366118c4f81c39d091b2e1b1d34120625ce34fca7c755aefeb363a0d9c8152b5ae0112d6",
    "659656da49c802715847c9a18ef3fae92b877ccb483e338eaacfeff2a15706d7bcfff8d957d4fb69f135dc7cb87cf4612bb192a5225a41dd6532895aabcaa4c9"
    "fcb6e7a3b87db4ea50272a2f0b969e035a51ab4045cac0b84473605ceb71a7fe79d786cfc05175b1e6a1570d7cf3fdfdb745acebd0e6526d16279fd1dcee9f01",
    "21a4b6ea548ae657e95a4838ca6f516a36e61efffcdfe7a9d33a5fb3d167e09a42cbaa48eb269eeb78395408b38e92431d76eb4f16d6cba35c7fde43f2997154"
    "5d7ec947bf30b8e6fb77ccfef9e247e5324e3ea3b9dd3fe307692e7a44c7e285687b434db2f7b983ed38e5f9e7b8e7e959f5f92cf1405f0f51a076384abef3fa",
    "e48943be5b483e338ecabf7eff38f7e59fbc7df93fe5c3e58b3b3bb2120ba99b646d25d3e077727c424a673d5f74932fe647c93781be98f77cd13d7cb87c917b"
    "ba4785b29140392529d9649bd94ab5c230eaf922aefdb6d373ca394b6cd567e01d7fe89a7392d614a1353e9f7cee90ef3692cf8cffc12787f2b02c797ee90e3e",
    "5c7eb92a938904fda8093699507d6d2f1c09e6a2828bfedf99b4f939c2f56341efa778fd58f0d68feee1c3b67ecc372a5c9ae49a3149aba8d461637d7f25efa2"
    "f786a6d80fbd7346cf0f5dc3e79d338e66fc29f643ef7cd1f343d7f079e78ba319dfede78bf396d8aacfc0858e1c415605a67fc038a9eff1dc41f29971543d2c",
    "7930cae1f9a64bf870f9e69e466e45c934bf2f05e4f5c8ea7128c208d005fbea9f237e868d",
    "" };

  c3_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c3_data[0], 14792U, &c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_eML_blk_kernel(SFc3_LaneDetectionInstanceStruct *chartInstance,
  real32_T c3_b_laneNetOut[6], boolean_T *c3_b_laneFound, real32_T c3_b_ltPts[56],
  real32_T c3_b_rtPts[56])
{
  static real_T c3_dv[6] = { -0.0002, 0.0002, 1.474, -0.0002, 0.0045, -1.3787 };

  static real_T c3_dv1[6] = { 0.003, 0.0766, 0.6313, 0.0026, 0.0736, 0.9846 };

  real_T c3_Tinv[9];
  real_T c3_x[9];
  real_T c3_absx11;
  real_T c3_absx21;
  real_T c3_absx31;
  real_T c3_b_absx11;
  real_T c3_b_absx21;
  real_T c3_b_absx31;
  real_T c3_b_t1;
  real_T c3_b_t2;
  real_T c3_b_t3;
  real_T c3_b_z;
  real_T c3_c_z;
  real_T c3_d_z;
  real_T c3_e_z;
  real_T c3_f_z;
  real_T c3_t1;
  real_T c3_t2;
  real_T c3_t3;
  real_T c3_z;
  int32_T c3_b_itmp;
  int32_T c3_b_k;
  int32_T c3_b_p1;
  int32_T c3_b_p2;
  int32_T c3_b_p3;
  int32_T c3_itmp;
  int32_T c3_k;
  int32_T c3_p1;
  int32_T c3_p2;
  int32_T c3_p3;
  real32_T c3_params[6];
  boolean_T c3_Tinv_dirtyOnCpu;
  boolean_T c3_c_laneFound;
  boolean_T c3_ltPts_dirtyOnGpu;
  boolean_T c3_params_dirtyOnGpu;
  boolean_T c3_rtPts_dirtyOnGpu;
  boolean_T c3_x_dirtyOnCpu;
  c3_rtPts_dirtyOnGpu = false;
  c3_ltPts_dirtyOnGpu = false;
  if (!chartInstance->c3_laneCoeffMeans_not_empty) {
    cudaMemcpy(chartInstance->c3_gpu_dv, &c3_dv[0], 48UL, cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(c3_gpu_laneCoeffMeans, chartInstance->c3_laneCoeffMeans,
                       48UL, 0UL, cudaMemcpyHostToDevice);
    c3_eML_blk_kernel_kernel1<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c3_gpu_dv);
    cudaMemcpyFromSymbol(chartInstance->c3_laneCoeffMeans, c3_gpu_laneCoeffMeans,
                         48UL, 0UL, cudaMemcpyDeviceToHost);
    chartInstance->c3_laneCoeffMeans_not_empty = true;
  }

  if (!chartInstance->c3_laneCoeffStds_not_empty) {
    cudaMemcpy(chartInstance->c3_gpu_dv1, &c3_dv1[0], 48UL,
               cudaMemcpyHostToDevice);
    cudaMemcpyToSymbol(c3_gpu_laneCoeffStds, chartInstance->c3_laneCoeffStds,
                       48UL, 0UL, cudaMemcpyHostToDevice);
    c3_eML_blk_kernel_kernel2<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c3_gpu_dv1);
    cudaMemcpyFromSymbol(chartInstance->c3_laneCoeffStds, c3_gpu_laneCoeffStds,
                         48UL, 0UL, cudaMemcpyDeviceToHost);
    chartInstance->c3_laneCoeffStds_not_empty = true;
  }

  cudaMemcpy(chartInstance->c3_gpu_laneNetOut, &c3_b_laneNetOut[0], 24UL,
             cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(c3_gpu_laneCoeffMeans, chartInstance->c3_laneCoeffMeans,
                     48UL, 0UL, cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(c3_gpu_laneCoeffStds, chartInstance->c3_laneCoeffStds, 48UL,
                     0UL, cudaMemcpyHostToDevice);
  c3_eML_blk_kernel_kernel3<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*chartInstance->c3_gpu_laneNetOut, *chartInstance->c3_c_gpu_params);
  if (!chartInstance->c3_vehicleXPoints_not_empty) {
    cudaMemcpyToSymbol(c3_gpu_vehicleXPoints, chartInstance->c3_vehicleXPoints,
                       224UL, 0UL, cudaMemcpyHostToDevice);
    c3_eML_blk_kernel_kernel4<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>();
    cudaMemcpyFromSymbol(chartInstance->c3_vehicleXPoints, c3_gpu_vehicleXPoints,
                         224UL, 0UL, cudaMemcpyDeviceToHost);
    chartInstance->c3_vehicleXPoints_not_empty = true;
  }

  cudaMemcpy(&c3_params[0], chartInstance->c3_c_gpu_params, 24UL,
             cudaMemcpyDeviceToHost);
  c3_params_dirtyOnGpu = false;
  if (muSingleScalarAbs(c3_params[5]) > 0.5F) {
    if (c3_params_dirtyOnGpu) {
      cudaMemcpy(&c3_params[0], chartInstance->c3_c_gpu_params, 24UL,
                 cudaMemcpyDeviceToHost);
    }

    if (muSingleScalarAbs(c3_params[2]) > 0.5F) {
      c3_eML_blk_kernel_kernel5<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_c_gpu_params, *chartInstance->c3_gpu_rt_y);
      for (c3_k = 0; c3_k < 2; c3_k++) {
        cudaMemcpyToSymbol(c3_gpu_vehicleXPoints,
                           chartInstance->c3_vehicleXPoints, 224UL, 0UL,
                           cudaMemcpyHostToDevice);
        c3_eML_blk_kernel_kernel6<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (c3_params[c3_k + 4], *chartInstance->c3_gpu_rt_y);
      }

      c3_eML_blk_kernel_kernel7<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_c_gpu_params, *chartInstance->c3_gpu_lt_y);
      for (c3_b_k = 0; c3_b_k < 2; c3_b_k++) {
        cudaMemcpyToSymbol(c3_gpu_vehicleXPoints,
                           chartInstance->c3_vehicleXPoints, 224UL, 0UL,
                           cudaMemcpyHostToDevice);
        c3_eML_blk_kernel_kernel8<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (c3_params[c3_b_k + 1], *chartInstance->c3_gpu_lt_y);
      }

      c3_Tinv[3] = -0.0032316839464807288;
      c3_Tinv[4] = -1.2852132429203174E-19;
      c3_Tinv[5] = 1.0305949982581226;
      c3_Tinv[6] = 1.9916790026632809E-35;
      c3_Tinv[7] = 0.0012931719938928032;
      c3_Tinv[8] = -0.22205377950113064;
      c3_Tinv[0] = -1.9788357004567556E-19;
      c3_Tinv[1] = -0.00070281981464454381;
      c3_Tinv[2] = 1.1512965678044422;
      cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                 cudaMemcpyHostToDevice);
      c3_Tinv_dirtyOnCpu = false;
      c3_eML_blk_kernel_kernel9<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
      c3_p1 = 0;
      c3_p2 = 3;
      c3_p3 = 6;
      c3_absx11 = muDoubleScalarAbs(c3_Tinv[0]);
      c3_absx21 = muDoubleScalarAbs(c3_Tinv[1]);
      c3_absx31 = muDoubleScalarAbs(c3_Tinv[2]);
      if ((c3_absx21 > c3_absx11) && (c3_absx21 > c3_absx31)) {
        c3_p1 = 3;
        c3_p2 = 0;
        if (c3_Tinv_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                     cudaMemcpyHostToDevice);
          c3_Tinv_dirtyOnCpu = false;
        }

        c3_eML_blk_kernel_kernel11<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
      } else {
        if (c3_absx31 > c3_absx11) {
          c3_p1 = 6;
          c3_p3 = 0;
          if (c3_Tinv_dirtyOnCpu) {
            cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                       cudaMemcpyHostToDevice);
            c3_Tinv_dirtyOnCpu = false;
          }

          c3_eML_blk_kernel_kernel10<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
        }
      }

      cudaMemcpy(&c3_x[0], chartInstance->c3_gpu_x, 72UL, cudaMemcpyDeviceToHost);
      c3_z = c3_x[1] / c3_x[0];
      c3_x[1] /= c3_x[0];
      c3_b_z = c3_x[2] / c3_x[0];
      c3_x[2] /= c3_x[0];
      c3_x[4] -= c3_z * c3_x[3];
      c3_x[5] -= c3_b_z * c3_x[3];
      c3_x[7] -= c3_z * c3_x[6];
      c3_x[8] -= c3_b_z * c3_x[6];
      if (muDoubleScalarAbs(c3_x[5]) > muDoubleScalarAbs(c3_x[4])) {
        c3_itmp = c3_p2;
        c3_p2 = c3_p3;
        c3_p3 = c3_itmp;
        c3_x[1] = c3_b_z;
        c3_x[2] = c3_z;
        c3_t1 = c3_x[4];
        c3_x[4] = c3_x[5];
        c3_x[5] = c3_t1;
        c3_t1 = c3_x[7];
        c3_x[7] = c3_x[8];
        c3_x[8] = c3_t1;
      }

      c3_c_z = c3_x[5] / c3_x[4];
      c3_x[5] /= c3_x[4];
      c3_x[8] -= c3_c_z * c3_x[7];
      c3_t3 = (c3_x[5] * c3_x[1] - c3_x[2]) / c3_x[8];
      c3_t2 = -(c3_x[1] + c3_x[7] * c3_t3) / c3_x[4];
      cudaMemcpy(chartInstance->c3_gpu_x, &c3_x[0], 72UL, cudaMemcpyHostToDevice);
      c3_x_dirtyOnCpu = false;
      c3_eML_blk_kernel_kernel12<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(c3_t3,
        c3_t2, *chartInstance->c3_gpu_x, c3_p1, *chartInstance->c3_b_gpu_Tinv);
      c3_t3 = -c3_x[5] / c3_x[8];
      c3_t2 = (1.0 - c3_x[7] * c3_t3) / c3_x[4];
      if (c3_x_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c3_gpu_x, &c3_x[0], 72UL,
                   cudaMemcpyHostToDevice);
      }

      c3_eML_blk_kernel_kernel13<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(c3_t3,
        c3_t2, *chartInstance->c3_gpu_x, c3_p2, *chartInstance->c3_b_gpu_Tinv);
      c3_t3 = 1.0 / c3_x[8];
      c3_t2 = -c3_x[7] * c3_t3 / c3_x[4];
      c3_eML_blk_kernel_kernel14<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(c3_t3,
        c3_t2, *chartInstance->c3_gpu_x, c3_p3, *chartInstance->c3_b_gpu_Tinv);
      cudaMemcpyToSymbol(c3_gpu_vehicleXPoints, chartInstance->c3_vehicleXPoints,
                         224UL, 0UL, cudaMemcpyHostToDevice);
      c3_eML_blk_kernel_kernel15<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_lt_y, *chartInstance->c3_gpu_fv);
      c3_eML_blk_kernel_kernel16<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>
        (*chartInstance->c3_b_gpu_Tinv, *chartInstance->c3_gpu_fv,
         *chartInstance->c3_c_gpu_U);
      c3_eML_blk_kernel_kernel17<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_U, *chartInstance->c3_c_gpu_U,
         *chartInstance->c3_gpu_b);
      c3_eML_blk_kernel_kernel18<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
        (*chartInstance->c3_c_gpu_U, *chartInstance->c3_gpu_ltPts);
      c3_ltPts_dirtyOnGpu = true;
      if (c3_Tinv_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                   cudaMemcpyHostToDevice);
      }

      c3_eML_blk_kernel_kernel19<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
      c3_b_p1 = 0;
      c3_b_p2 = 3;
      c3_b_p3 = 6;
      c3_b_absx11 = muDoubleScalarAbs(c3_Tinv[0]);
      c3_b_absx21 = muDoubleScalarAbs(c3_Tinv[1]);
      c3_b_absx31 = muDoubleScalarAbs(c3_Tinv[2]);
      if ((c3_b_absx21 > c3_b_absx11) && (c3_b_absx21 > c3_b_absx31)) {
        c3_b_p1 = 3;
        c3_b_p2 = 0;
        c3_eML_blk_kernel_kernel21<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
      } else {
        if (c3_b_absx31 > c3_b_absx11) {
          c3_b_p1 = 6;
          c3_b_p3 = 0;
          c3_eML_blk_kernel_kernel20<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
        }
      }

      cudaMemcpy(&c3_x[0], chartInstance->c3_gpu_x, 72UL, cudaMemcpyDeviceToHost);
      c3_d_z = c3_x[1] / c3_x[0];
      c3_x[1] /= c3_x[0];
      c3_e_z = c3_x[2] / c3_x[0];
      c3_x[2] /= c3_x[0];
      c3_x[4] -= c3_d_z * c3_x[3];
      c3_x[5] -= c3_e_z * c3_x[3];
      c3_x[7] -= c3_d_z * c3_x[6];
      c3_x[8] -= c3_e_z * c3_x[6];
      if (muDoubleScalarAbs(c3_x[5]) > muDoubleScalarAbs(c3_x[4])) {
        c3_b_itmp = c3_b_p2;
        c3_b_p2 = c3_b_p3;
        c3_b_p3 = c3_b_itmp;
        c3_x[1] = c3_e_z;
        c3_x[2] = c3_d_z;
        c3_b_t1 = c3_x[4];
        c3_x[4] = c3_x[5];
        c3_x[5] = c3_b_t1;
        c3_b_t1 = c3_x[7];
        c3_x[7] = c3_x[8];
        c3_x[8] = c3_b_t1;
      }

      c3_f_z = c3_x[5] / c3_x[4];
      c3_x[5] /= c3_x[4];
      c3_x[8] -= c3_f_z * c3_x[7];
      c3_b_t3 = (c3_x[5] * c3_x[1] - c3_x[2]) / c3_x[8];
      c3_b_t2 = -(c3_x[1] + c3_x[7] * c3_b_t3) / c3_x[4];
      c3_Tinv[c3_b_p1] = ((1.0 - c3_x[3] * c3_b_t2) - c3_x[6] * c3_b_t3) / c3_x
        [0];
      c3_Tinv[c3_b_p1 + 1] = c3_b_t2;
      c3_Tinv[c3_b_p1 + 2] = c3_b_t3;
      c3_b_t3 = -c3_x[5] / c3_x[8];
      c3_b_t2 = (1.0 - c3_x[7] * c3_b_t3) / c3_x[4];
      c3_Tinv[c3_b_p2] = -(c3_x[3] * c3_b_t2 + c3_x[6] * c3_b_t3) / c3_x[0];
      c3_Tinv[c3_b_p2 + 1] = c3_b_t2;
      c3_Tinv[c3_b_p2 + 2] = c3_b_t3;
      c3_b_t3 = 1.0 / c3_x[8];
      c3_b_t2 = -c3_x[7] * c3_b_t3 / c3_x[4];
      c3_Tinv[c3_b_p3] = -(c3_x[3] * c3_b_t2 + c3_x[6] * c3_b_t3) / c3_x[0];
      cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                 cudaMemcpyHostToDevice);
      c3_eML_blk_kernel_kernel22<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (c3_b_t3, c3_b_t2, c3_b_p3, *chartInstance->c3_gpu_Tinv);
      cudaMemcpyToSymbol(c3_gpu_vehicleXPoints, chartInstance->c3_vehicleXPoints,
                         224UL, 0UL, cudaMemcpyHostToDevice);
      c3_eML_blk_kernel_kernel23<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_rt_y, *chartInstance->c3_gpu_fv1);
      c3_eML_blk_kernel_kernel24<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_fv1,
         *chartInstance->c3_c_gpu_U);
      c3_eML_blk_kernel_kernel25<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
        (*chartInstance->c3_b_gpu_U, *chartInstance->c3_c_gpu_U,
         *chartInstance->c3_gpu_b);
      c3_eML_blk_kernel_kernel26<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
        (*chartInstance->c3_c_gpu_U, *chartInstance->c3_gpu_rtPts);
      c3_rtPts_dirtyOnGpu = true;
      c3_c_laneFound = true;
    } else {
      c3_c_laneFound = false;
    }
  } else {
    c3_c_laneFound = false;
  }

  *c3_b_laneFound = c3_c_laneFound;
  if (c3_ltPts_dirtyOnGpu) {
    cudaMemcpy(&c3_b_ltPts[0], chartInstance->c3_gpu_ltPts, 224UL,
               cudaMemcpyDeviceToHost);
  }

  if (c3_rtPts_dirtyOnGpu) {
    cudaMemcpy(&c3_b_rtPts[0], chartInstance->c3_gpu_rtPts, 224UL,
               cudaMemcpyDeviceToHost);
  }
}

static boolean_T c3_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_laneFound, const char_T *c3_identifier)
{
  emlrtMsgIdentifier c3_thisId;
  boolean_T c3_y;
  c3_thisId.fIdentifier = const_cast<const char_T *>(c3_identifier);
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_y = c3_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_laneFound),
    &c3_thisId);
  sf_mex_destroy(&c3_b_laneFound);
  return c3_y;
}

static boolean_T c3_b_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  boolean_T c3_b;
  boolean_T c3_y;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_b, 1, 11, 0U, 0, 0U, 0);
  c3_y = c3_b;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static void c3_c_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_ltPts, const char_T *c3_identifier,
  real32_T c3_y[56])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = const_cast<const char_T *>(c3_identifier);
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_ltPts), &c3_thisId, c3_y);
  sf_mex_destroy(&c3_b_ltPts);
}

static void c3_d_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  real32_T c3_y[56])
{
  int32_T c3_i;
  real32_T c3_fv[56];
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_fv, 0, 1, 0U, 1, 0U, 2, 28, 2);
  for (c3_i = 0; c3_i < 56; c3_i++) {
    c3_y[c3_i] = c3_fv[c3_i];
  }

  sf_mex_destroy(&c3_u);
}

static void c3_e_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_laneCoeffMeans, const char_T
  *c3_identifier, boolean_T *c3_svPtr, real_T c3_y[6])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = const_cast<const char_T *>(c3_identifier);
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_f_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_laneCoeffMeans),
                        &c3_thisId, c3_svPtr, c3_y);
  sf_mex_destroy(&c3_b_laneCoeffMeans);
}

static void c3_f_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T *c3_svPtr, real_T c3_y[6])
{
  real_T c3_dv[6];
  int32_T c3_i;
  if (mxIsEmpty(c3_u)) {
    *c3_svPtr = false;
  } else {
    *c3_svPtr = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv, 1, 0, 0U, 1, 0U, 2, 1, 6);
    for (c3_i = 0; c3_i < 6; c3_i++) {
      c3_y[c3_i] = c3_dv[c3_i];
    }
  }

  sf_mex_destroy(&c3_u);
}

static void c3_g_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_vehicleXPoints, const char_T
  *c3_identifier, boolean_T *c3_svPtr, real_T c3_y[28])
{
  emlrtMsgIdentifier c3_thisId;
  c3_thisId.fIdentifier = const_cast<const char_T *>(c3_identifier);
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_h_emlrt_marshallIn(chartInstance, sf_mex_dup(c3_b_vehicleXPoints),
                        &c3_thisId, c3_svPtr, c3_y);
  sf_mex_destroy(&c3_b_vehicleXPoints);
}

static void c3_h_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId,
  boolean_T *c3_svPtr, real_T c3_y[28])
{
  real_T c3_dv[28];
  int32_T c3_i;
  if (mxIsEmpty(c3_u)) {
    *c3_svPtr = false;
  } else {
    *c3_svPtr = true;
    sf_mex_import(c3_parentId, sf_mex_dup(c3_u), c3_dv, 1, 0, 0U, 1, 0U, 2, 1,
                  28);
    for (c3_i = 0; c3_i < 28; c3_i++) {
      c3_y[c3_i] = c3_dv[c3_i];
    }
  }

  sf_mex_destroy(&c3_u);
}

static uint8_T c3_i_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_b_is_active_c3_LaneDetection, const char_T
  *c3_identifier)
{
  emlrtMsgIdentifier c3_thisId;
  uint8_T c3_y;
  c3_thisId.fIdentifier = const_cast<const char_T *>(c3_identifier);
  c3_thisId.fParent = NULL;
  c3_thisId.bParentIsCell = false;
  c3_y = c3_j_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c3_b_is_active_c3_LaneDetection), &c3_thisId);
  sf_mex_destroy(&c3_b_is_active_c3_LaneDetection);
  return c3_y;
}

static uint8_T c3_j_emlrt_marshallIn(SFc3_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c3_u, const emlrtMsgIdentifier *c3_parentId)
{
  uint8_T c3_b_u;
  uint8_T c3_y;
  sf_mex_import(c3_parentId, sf_mex_dup(c3_u), &c3_b_u, 1, 3, 0U, 0, 0U, 0);
  c3_y = c3_b_u;
  sf_mex_destroy(&c3_u);
  return c3_y;
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel1(const
  real_T c3_dv[6])
{
  int32_T c3_i4;
  c3_i4 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i4 < 6) {
    c3_gpu_laneCoeffMeans[c3_i4] = c3_dv[c3_i4];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel2(const
  real_T c3_dv1[6])
{
  int32_T c3_i5;
  c3_i5 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i5 < 6) {
    c3_gpu_laneCoeffStds[c3_i5] = c3_dv1[c3_i5];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel3(const
  real32_T c3_b_laneNetOut[6], real32_T c3_params[6])
{
  int32_T c3_i6;
  c3_i6 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i6 < 6) {
    c3_params[c3_i6] = c3_b_laneNetOut[c3_i6] * (real32_T)
      c3_gpu_laneCoeffStds[c3_i6] + (real32_T)c3_gpu_laneCoeffMeans[c3_i6];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel4()
{
  int32_T c3_i7;
  c3_i7 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i7 < 28) {
    c3_gpu_vehicleXPoints[c3_i7] = (real_T)c3_i7 + 3.0;
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel5(const
  real32_T c3_params[6], real32_T c3_rt_y[28])
{
  int32_T c3_i8;
  c3_i8 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i8 < 28) {
    c3_rt_y[c3_i8] = c3_params[3];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel6(const
  real32_T c3_params, real32_T c3_rt_y[28])
{
  int32_T c3_i10;
  c3_i10 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i10 < 28) {
    c3_rt_y[c3_i10] = (real32_T)c3_gpu_vehicleXPoints[c3_i10] * c3_rt_y[c3_i10]
      + c3_params;
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel7(const
  real32_T c3_params[6], real32_T c3_lt_y[28])
{
  int32_T c3_i9;
  c3_i9 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i9 < 28) {
    c3_lt_y[c3_i9] = c3_params[0];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel8(const
  real32_T c3_params, real32_T c3_lt_y[28])
{
  int32_T c3_i11;
  c3_i11 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i11 < 28) {
    c3_lt_y[c3_i11] = (real32_T)c3_gpu_vehicleXPoints[c3_i11] * c3_lt_y[c3_i11]
      + c3_params;
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel9(const
  real_T c3_Tinv[9], real_T c3_x[9])
{
  int32_T c3_i12;
  c3_i12 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i12 < 9) {
    c3_x[c3_i12] = c3_Tinv[c3_i12];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel10(const
  real_T c3_Tinv[9], real_T c3_x[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_x[0] = c3_Tinv[2];
    c3_x[2] = c3_Tinv[0];
    c3_x[3] = c3_Tinv[5];
    c3_x[5] = c3_Tinv[3];
    c3_x[6] = c3_Tinv[8];
    c3_x[8] = c3_Tinv[6];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel11(const
  real_T c3_Tinv[9], real_T c3_x[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_x[0] = c3_Tinv[1];
    c3_x[1] = c3_Tinv[0];
    c3_x[3] = c3_Tinv[4];
    c3_x[4] = c3_Tinv[3];
    c3_x[6] = c3_Tinv[7];
    c3_x[7] = c3_Tinv[6];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel12(const
  real_T c3_t3, const real_T c3_t2, const real_T c3_x[9], const int32_T c3_p1,
  real_T c3_Tinv[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_Tinv[c3_p1] = ((1.0 - c3_x[3] * c3_t2) - c3_x[6] * c3_t3) / c3_x[0];
    c3_Tinv[c3_p1 + 1] = c3_t2;
    c3_Tinv[c3_p1 + 2] = c3_t3;
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel13(const
  real_T c3_t3, const real_T c3_t2, const real_T c3_x[9], const int32_T c3_p2,
  real_T c3_Tinv[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_Tinv[c3_p2] = -(c3_x[3] * c3_t2 + c3_x[6] * c3_t3) / c3_x[0];
    c3_Tinv[c3_p2 + 1] = c3_t2;
    c3_Tinv[c3_p2 + 2] = c3_t3;
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel14(const
  real_T c3_t3, const real_T c3_t2, const real_T c3_x[9], const int32_T c3_p3,
  real_T c3_Tinv[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_Tinv[c3_p3] = -(c3_x[3] * c3_t2 + c3_x[6] * c3_t3) / c3_x[0];
    c3_Tinv[c3_p3 + 1] = c3_t2;
    c3_Tinv[c3_p3 + 2] = c3_t3;
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel15(const
  real32_T c3_lt_y[28], real32_T c3_fv[84])
{
  int32_T c3_i13;
  c3_i13 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i13 < 28) {
    c3_fv[c3_i13] = (real32_T)c3_gpu_vehicleXPoints[c3_i13];
    c3_fv[c3_i13 + 28] = c3_lt_y[c3_i13];
    c3_fv[c3_i13 + 56] = 1.0F;
  }
}

static __global__ __launch_bounds__(96, 1) void c3_eML_blk_kernel_kernel16(const
  real_T c3_Tinv[9], const real32_T c3_fv[84], real32_T c3_U[84])
{
  uint64_T c3_threadId;
  int32_T c3_i14;
  int32_T c3_i15;
  int32_T c3_i16;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_i16 = (int32_T)(c3_threadId % 3UL);
  c3_i14 = (int32_T)((c3_threadId - (uint64_T)c3_i16) / 3UL);
  if ((c3_i14 < 28) && (c3_i16 < 3)) {
    c3_U[c3_i14 + 28 * c3_i16] = 0.0F;
    for (c3_i15 = 0; c3_i15 < 3; c3_i15++) {
      c3_U[c3_i14 + 28 * c3_i16] += c3_fv[c3_i14 + 28 * c3_i15] * (real32_T)
        c3_Tinv[c3_i15 + 3 * c3_i16];
    }
  }
}

static __global__ __launch_bounds__(64, 1) void c3_eML_blk_kernel_kernel17
  (real32_T c3_U[56], real32_T c3_b_U[84], real32_T c3_b[56])
{
  uint64_T c3_threadId;
  int32_T c3_jtilecol;
  int32_T c3_k;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_k = (int32_T)(c3_threadId % 28UL);
  c3_jtilecol = (int32_T)((c3_threadId - (uint64_T)c3_k) / 28UL);
  if ((c3_jtilecol < 2) && (c3_k < 28)) {
    c3_b[c3_jtilecol * 28 + c3_k] = c3_b_U[c3_k + 56];
    c3_U[c3_k + 28 * c3_jtilecol] = c3_b_U[c3_k + 28 * c3_jtilecol] / c3_b[c3_k
      + 28 * c3_jtilecol];
    c3_b_U[c3_k + 28 * c3_jtilecol] = c3_U[c3_k + 28 * c3_jtilecol];
  }
}

static __global__ __launch_bounds__(64, 1) void c3_eML_blk_kernel_kernel18(const
  real32_T c3_U[84], real32_T c3_b_ltPts[56])
{
  uint64_T c3_threadId;
  int32_T c3_i17;
  int32_T c3_i19;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_i19 = (int32_T)(c3_threadId % 28UL);
  c3_i17 = (int32_T)((c3_threadId - (uint64_T)c3_i19) / 28UL);
  if ((c3_i17 < 2) && (c3_i19 < 28)) {
    c3_b_ltPts[c3_i19 + 28 * c3_i17] = c3_U[c3_i19 + 28 * c3_i17];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel19(const
  real_T c3_Tinv[9], real_T c3_x[9])
{
  int32_T c3_i18;
  c3_i18 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i18 < 9) {
    c3_x[c3_i18] = c3_Tinv[c3_i18];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel20(const
  real_T c3_Tinv[9], real_T c3_x[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_x[0] = c3_Tinv[2];
    c3_x[2] = c3_Tinv[0];
    c3_x[3] = c3_Tinv[5];
    c3_x[5] = c3_Tinv[3];
    c3_x[6] = c3_Tinv[8];
    c3_x[8] = c3_Tinv[6];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel21(const
  real_T c3_Tinv[9], real_T c3_x[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_x[0] = c3_Tinv[1];
    c3_x[1] = c3_Tinv[0];
    c3_x[3] = c3_Tinv[4];
    c3_x[4] = c3_Tinv[3];
    c3_x[6] = c3_Tinv[7];
    c3_x[7] = c3_Tinv[6];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel22(const
  real_T c3_t3, const real_T c3_t2, const int32_T c3_p3, real_T c3_Tinv[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_Tinv[c3_p3 + 1] = c3_t2;
    c3_Tinv[c3_p3 + 2] = c3_t3;
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel23(const
  real32_T c3_rt_y[28], real32_T c3_fv1[84])
{
  int32_T c3_i20;
  c3_i20 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i20 < 28) {
    c3_fv1[c3_i20] = (real32_T)c3_gpu_vehicleXPoints[c3_i20];
    c3_fv1[c3_i20 + 28] = c3_rt_y[c3_i20];
    c3_fv1[c3_i20 + 56] = 1.0F;
  }
}

static __global__ __launch_bounds__(96, 1) void c3_eML_blk_kernel_kernel24(const
  real_T c3_Tinv[9], const real32_T c3_fv1[84], real32_T c3_U[84])
{
  uint64_T c3_threadId;
  int32_T c3_i21;
  int32_T c3_i22;
  int32_T c3_i23;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_i23 = (int32_T)(c3_threadId % 3UL);
  c3_i21 = (int32_T)((c3_threadId - (uint64_T)c3_i23) / 3UL);
  if ((c3_i21 < 28) && (c3_i23 < 3)) {
    c3_U[c3_i21 + 28 * c3_i23] = 0.0F;
    for (c3_i22 = 0; c3_i22 < 3; c3_i22++) {
      c3_U[c3_i21 + 28 * c3_i23] += c3_fv1[c3_i21 + 28 * c3_i22] * (real32_T)
        c3_Tinv[c3_i22 + 3 * c3_i23];
    }
  }
}

static __global__ __launch_bounds__(64, 1) void c3_eML_blk_kernel_kernel25
  (real32_T c3_U[56], real32_T c3_b_U[84], real32_T c3_b[56])
{
  uint64_T c3_threadId;
  int32_T c3_jtilecol;
  int32_T c3_k;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_k = (int32_T)(c3_threadId % 28UL);
  c3_jtilecol = (int32_T)((c3_threadId - (uint64_T)c3_k) / 28UL);
  if ((c3_jtilecol < 2) && (c3_k < 28)) {
    c3_b[c3_jtilecol * 28 + c3_k] = c3_b_U[c3_k + 56];
    c3_U[c3_k + 28 * c3_jtilecol] = c3_b_U[c3_k + 28 * c3_jtilecol] / c3_b[c3_k
      + 28 * c3_jtilecol];
    c3_b_U[c3_k + 28 * c3_jtilecol] = c3_U[c3_k + 28 * c3_jtilecol];
  }
}

static __global__ __launch_bounds__(64, 1) void c3_eML_blk_kernel_kernel26(const
  real32_T c3_U[84], real32_T c3_b_rtPts[56])
{
  uint64_T c3_threadId;
  int32_T c3_i24;
  int32_T c3_i25;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_i25 = (int32_T)(c3_threadId % 28UL);
  c3_i24 = (int32_T)((c3_threadId - (uint64_T)c3_i25) / 28UL);
  if ((c3_i24 < 2) && (c3_i25 < 28)) {
    c3_b_rtPts[c3_i25 + 28 * c3_i24] = c3_U[c3_i25 + 28 * c3_i24];
  }
}

static void init_dsm_address_info(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static void init_simulink_io_address(SFc3_LaneDetectionInstanceStruct
  *chartInstance)
{
  chartInstance->c3_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c3_laneNetOut = (real32_T (*)[6])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c3_laneFound = (boolean_T *)ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c3_ltPts = (real32_T (*)[56])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c3_rtPts = (real32_T (*)[56])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 3);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c3_LaneDetection_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2051119261U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(4225097776U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1871082434U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2287114376U);
}

mxArray *sf_c3_LaneDetection_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,0);
  return(mxcell3p);
}

mxArray *sf_c3_LaneDetection_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("GPUAcceleration");
  mxArray *hiddenFallbackType = mxCreateString("none");
  mxArray *hiddenFallbackReason = mxCreateString("");
  mxArray *incompatibleSymbol = mxCreateString("");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c3_LaneDetection_updateBuildInfo_args_info(void)
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

static const mxArray *sf_get_sim_state_info_c3_LaneDetection(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNrtlkFLwzAUx9OyTXtY6cGDiId9Aot48WbEIRQcFLx4qyFNaaFNapsNveoX2Efxo/kRTNps1lA"
    "suOImGng83oP/S3/hn1BgeDMgli3i1QJgJPK+CBPUa6hqQ8SeynV/sO5PRPCnnMh+WWAvFJmirK"
    "rR/NGjEavmn4OP+aOW+UZjvqX69Xq72Ex/AKUeNvSDFr3V0DuqThEl12xOQ9W3G3l7PIdQ17fxD"
    "DUeWafc5+XOcBx/m6PolcOBm+nr/bv8ZWsctvLXFSNRNCOIlorD+PoeGto9NFUtNSlIQN7yPW08"
    "psbjrGkCWCWIYB/+iDvO5VQ7F1m7McuIG2c4dS+nmFFesNSVhxWEhBPME0YDzFgRJhRxUp5kv8g"
    "HY4133PTBLQ9Xvt66D5Z57YP7h38f/NB7sCBxglNy57OE8l15D5bPtQ/Ayx/zwaQXH/gdvEcar6"
    "yTMkCCakECfBbcCM7pCvPzf8g7obOKrw=="
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c3_LaneDetection_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "sc88SziHwZbSUvEPWN0mVPE";
}

static void sf_opaque_initialize_c3_LaneDetection(void *chartInstanceVar)
{
  initialize_params_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
    chartInstanceVar);
  initialize_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c3_LaneDetection(void *chartInstanceVar)
{
  enable_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c3_LaneDetection(void *chartInstanceVar)
{
  disable_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c3_LaneDetection(void *chartInstanceVar)
{
  sf_gateway_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c3_LaneDetection(SimStruct* S)
{
  return get_sim_state_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c3_LaneDetection(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_cleanup_runtime_resources_c3_LaneDetection(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc3_LaneDetectionInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_LaneDetection_optimization_info();
    }

    mdl_cleanup_runtime_resources_c3_LaneDetection
      ((SFc3_LaneDetectionInstanceStruct*) chartInstanceVar);
    ((SFc3_LaneDetectionInstanceStruct*) chartInstanceVar)->
      ~SFc3_LaneDetectionInstanceStruct();
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c3_LaneDetection(void *chartInstanceVar)
{
  mdl_start_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_mdl_terminate_c3_LaneDetection(void *chartInstanceVar)
{
  mdl_terminate_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c3_LaneDetection(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  sf_warn_if_symbolic_dimension_param_changed(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
      sf_get_chart_instance_ptr(S));
    initSimStructsc3_LaneDetection((SFc3_LaneDetectionInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

const char* sf_c3_LaneDetection_get_post_codegen_info(void)
{
  int i;
  const char* encStrCodegen [35] = {
    "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",
    "=",
    ""
  };

  static char newstr [2477] = "";
  newstr[0] = '\0';
  for (i = 0; i < 35; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c3_LaneDetection(SimStruct *S)
{
  const char* newstr = sf_c3_LaneDetection_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(916045491U));
  ssSetChecksum1(S,(1789172611U));
  ssSetChecksum2(S,(274519504U));
  ssSetChecksum3(S,(3822757219U));
}

static void mdlRTW_c3_LaneDetection(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c3_LaneDetection(SimStruct *S)
{
  SFc3_LaneDetectionInstanceStruct *chartInstance;
  chartInstance = (SFc3_LaneDetectionInstanceStruct *)utMalloc(sizeof
    (SFc3_LaneDetectionInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc3_LaneDetectionInstanceStruct));
  chartInstance = new (chartInstance) SFc3_LaneDetectionInstanceStruct;
  chartInstance->chartInfo.chartInstance = chartInstance;
  if (ssGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME && ssGetOffsetTime(S, 0) ==
      0 && ssGetNumContStates(ssGetRootSS(S)) > 0) {
    sf_error_out_about_continuous_sample_time_with_persistent_vars(S);
  }

  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c3_LaneDetection;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c3_LaneDetection;
  chartInstance->chartInfo.mdlStart = sf_opaque_mdl_start_c3_LaneDetection;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c3_LaneDetection;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c3_LaneDetection;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c3_LaneDetection;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c3_LaneDetection;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c3_LaneDetection;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c3_LaneDetection;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c3_LaneDetection;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c3_LaneDetection;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c3_LaneDetection;
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
    chartInstance->c3_JITStateAnimation,
    chartInstance->c3_JITTransitionAnimation);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c3_LaneDetection(chartInstance);
}

void c3_LaneDetection_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c3_LaneDetection(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c3_LaneDetection(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c3_LaneDetection(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c3_LaneDetection_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
