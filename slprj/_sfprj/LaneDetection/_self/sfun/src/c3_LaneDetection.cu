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
static __device__ real_T c3_gpu_laneCoeffStds[6];
static __device__ real_T c3_gpu_laneCoeffMeans[6];

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
static __global__ void c3_eML_blk_kernel_kernel9(const real_T c3_T[9], real_T
  c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel10(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel11(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel12(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel13(const real_T c3_t3, const
  real_T c3_t2, const real_T c3_x[9], const int32_T c3_p1, real_T c3_Tinv[9]);
static __global__ void c3_eML_blk_kernel_kernel14(const real_T c3_t3, const
  real_T c3_t2, const real_T c3_x[9], const int32_T c3_p2, real_T c3_Tinv[9]);
static __global__ void c3_eML_blk_kernel_kernel15(const real_T c3_t3, const
  real_T c3_t2, const real_T c3_x[9], const int32_T c3_p3, real_T c3_Tinv[9]);
static __global__ void c3_eML_blk_kernel_kernel16(const real32_T c3_lt_y[28],
  real32_T c3_fv[84]);
static __global__ void c3_eML_blk_kernel_kernel17(const real_T c3_Tinv[9], const
  real32_T c3_fv[84], real32_T c3_U[84]);
static __global__ void c3_eML_blk_kernel_kernel18(real32_T c3_U[56], real32_T
  c3_b_U[84], real32_T c3_b[56]);
static __global__ void c3_eML_blk_kernel_kernel19(const real32_T c3_U[84],
  real32_T c3_b_ltPts[56]);
static __global__ void c3_eML_blk_kernel_kernel20(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel21(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel22(const real_T c3_Tinv[9],
  real_T c3_x[9]);
static __global__ void c3_eML_blk_kernel_kernel23(const real_T c3_t3, const
  real_T c3_t2, const int32_T c3_p3, real_T c3_Tinv[9]);
static __global__ void c3_eML_blk_kernel_kernel24(const real32_T c3_rt_y[28],
  real32_T c3_fv1[84]);
static __global__ void c3_eML_blk_kernel_kernel25(const real_T c3_Tinv[9], const
  real32_T c3_fv1[84], real32_T c3_U[84]);
static __global__ void c3_eML_blk_kernel_kernel26(real32_T c3_U[56], real32_T
  c3_b_U[84], real32_T c3_b[56]);
static __global__ void c3_eML_blk_kernel_kernel27(const real32_T c3_U[84],
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
  cudaMalloc(&chartInstance->c3_c_gpu_params, 4UL);
  cudaMalloc(&chartInstance->c3_gpu_p2, 4UL);
  cudaMalloc(&chartInstance->c3_b_gpu_t2, 8UL);
  cudaMalloc(&chartInstance->c3_gpu_lt_y, 112UL);
  cudaMalloc(&chartInstance->c3_gpu_t3, 8UL);
  cudaMalloc(&chartInstance->c3_gpu_rtPts, 224UL);
  cudaMalloc(&chartInstance->c3_gpu_laneNetOut, 24UL);
  cudaMalloc(&chartInstance->c3_gpu_Tinv, 72UL);
  cudaMalloc(&chartInstance->c3_gpu_rt_y, 112UL);
  cudaMalloc(&chartInstance->c3_gpu_dv1, 48UL);
  cudaMalloc(&chartInstance->c3_gpu_t2, 8UL);
  cudaMalloc(&chartInstance->c3_b_gpu_params, 24UL);
  cudaMalloc(&chartInstance->c3_gpu_dv, 48UL);
  cudaMalloc(&chartInstance->c3_b_gpu_p3, 4UL);
  cudaMalloc(&chartInstance->c3_gpu_U, 224UL);
  cudaMalloc(&chartInstance->c3_b_gpu_Tinv, 72UL);
  cudaMalloc(&chartInstance->c3_gpu_fv, 336UL);
  cudaMalloc(&chartInstance->c3_gpu_ltPts, 224UL);
  cudaMalloc(&chartInstance->c3_b_gpu_U, 336UL);
  cudaMalloc(&chartInstance->c3_c_gpu_U, 224UL);
  cudaMalloc(&chartInstance->c3_gpu_p1, 4UL);
  cudaMalloc(&chartInstance->c3_gpu_fv1, 336UL);
  cudaMalloc(&chartInstance->c3_gpu_T, 72UL);
  cudaMalloc(&chartInstance->c3_b_gpu_t3, 8UL);
  cudaMalloc(&chartInstance->c3_gpu_b, 224UL);
  cudaMalloc(&chartInstance->c3_gpu_x, 72UL);
  cudaMalloc(&chartInstance->c3_gpu_p3, 4UL);
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
  cudaFree(chartInstance->c3_c_gpu_params);
  cudaFree(*chartInstance->c3_gpu_dv);
  cudaFree(*chartInstance->c3_gpu_ltPts);
  cudaFree(*chartInstance->c3_gpu_rt_y);
  cudaFree(*chartInstance->c3_c_gpu_U);
  cudaFree(*chartInstance->c3_b_gpu_U);
  cudaFree(chartInstance->c3_gpu_t2);
  cudaFree(*chartInstance->c3_gpu_b);
  cudaFree(*chartInstance->c3_b_gpu_params);
  cudaFree(*chartInstance->c3_gpu_lt_y);
  cudaFree(chartInstance->c3_gpu_params);
  cudaFree(chartInstance->c3_b_gpu_t3);
  cudaFree(*chartInstance->c3_b_gpu_Tinv);
  cudaFree(chartInstance->c3_b_gpu_p3);
  cudaFree(chartInstance->c3_b_gpu_t2);
  cudaFree(chartInstance->c3_gpu_p1);
  cudaFree(*chartInstance->c3_gpu_Tinv);
  cudaFree(*chartInstance->c3_gpu_T);
  cudaFree(*chartInstance->c3_gpu_rtPts);
  cudaFree(chartInstance->c3_gpu_t3);
  cudaFree(chartInstance->c3_gpu_p3);
  cudaFree(*chartInstance->c3_gpu_dv1);
  cudaFree(*chartInstance->c3_gpu_U);
  cudaFree(*chartInstance->c3_gpu_fv);
  cudaFree(*chartInstance->c3_gpu_laneNetOut);
  cudaFree(*chartInstance->c3_gpu_x);
  cudaFree(*chartInstance->c3_gpu_fv1);
  cudaFree(chartInstance->c3_gpu_p2);
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
    "789ced5adf6fd2401cef0c1a13e386899ab807df4cd4258063307952ca8fb93136b6026133064a3968b35e8fb50763beb847dff4c5e85f618cfe01d3bfc007ff"
    "16dfa5408556cf6e160f29bd841cdf7ce87d3efd7e7b9fdc1d65e6d6b3730cc3cc773fdf5e30cc8718d36b57fb1de31ff4171873b3e27383fe8a2536da45c667",
    "ba4ec7bf76f95e0d6201291874703f5078087e5e594350527805e78f9b80518186e436a8f590ba2483bc0401371a6ce9114c8f403f031dd2bf2744201c702dc8"
    "a8a23654288f06bd7ce8ad42b85f9f4d3eaccd9a0febef0cbe93bfe433c65fb4e13370995740b9063010b08494b280905aeba61903cdaca7e250cf25a29e3ea2",
    "494a430643be770ef956887c66fc69ea59504410044528c8c178527ffe542407c9790940fd3abbbc2c9c51a7b51ffefe72afbff5e6469526dfe7475f5668f219"
    "6d527c1dc278677dce6e12f8fc163c5dc8ee84ebd578a4f530c2458e500c24c5223bd491b3e1b1d3c110625ae34fcb7c755aefeb363a0d5c40b0d9c280452da5",
    "c6abc7595403f2a88e8a431de7f5d3d70ef902443e334eaacfeff2a15786d6bcff78ea2bebfdacf81a6d3e5a3ebad1c9840ae938ded3b6847ce6b0d31438b6f4"
    "c4f351da3eda74a8939417bfa56f005cc675a4c23c5a877c034c6a3deed43fefdbf01938a92ed63cf4ab416fbe7fa7bc0ef5374f0334f98ce676ff5485629b13",
    "0ee4dd940852da46683f510c2fa7dce39ffffb3c755ae76b36fa0c1cabbca2c9bc6ee2c5ae97239531eba838d461b7feaca156759cebcf25229f1927d5e5977c"
    "0c0a436bbebf7f4bd73f1f2f2ddea1c96734b7fb673215c225362356b9503aa53d1022cd6a6e37e9f9e7a4e7e959f5f92cf1505f1f5111de1f27df797df2c421",
    "df6d229f1927e55fbf7f9afbf24fdebefc9ff2d1f2c5cd4d454d46b5147b18cab7c5cda298815c21edf9a28b7cb1344ebe29f4c592e78beee1a3e58b8de7dbe9"
    "68211eaee6a05ac81e0b6bb94e0c253c5fa4b5df767a4e396f89adfa0cbceb0f3d73cef258953a93f3c9970ef9ee12f9ccf81f7c72240f01e8f9a53bf868f9e5",
    "b2c26632fcee114809d1d6ca762c1e29262417fdbf336df3738cebc73dbd9fe1f5e39eb77e740f1fb5f563a95d6f706ce32809715d4befb75777422517bd3734"
    "c37ee89d337a7ee81a3eef9c713ce3cfb01f7ae78b9e1fba86cf3b5f1ccff86e3f5f5cb0c4567d062e75e5488a26098303c6697d8fe71e91cf8c93ea61c98351",
    "0ecf375dc247cb37b731bb96603971078695d5f8f241342e48c805fbea1f34f98517", "" };

  c3_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c3_data[0], 14792U, &c3_nameCaptureInfo);
  return c3_nameCaptureInfo;
}

static void c3_eML_blk_kernel(SFc3_LaneDetectionInstanceStruct *chartInstance,
  real32_T c3_b_laneNetOut[6], boolean_T *c3_b_laneFound, real32_T c3_b_ltPts[56],
  real32_T c3_b_rtPts[56])
{
  static real_T c3_T[9] = { 318.9034, -309.4362, 1594.5169999999998, 257.5352,
    -1.5769482917388672E-14, 2037.9982547799998, 1.0, -6.123233995736766E-17,
    5.0 };

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
  real_T c3_c_t1;
  real_T c3_c_t2;
  real_T c3_c_t3;
  real_T c3_c_z;
  real_T c3_d_z;
  real_T c3_e_z;
  real_T c3_f_z;
  real_T c3_g_z;
  real_T c3_h_z;
  real_T c3_t1;
  real_T c3_t2;
  real_T c3_t3;
  real_T c3_z;
  int32_T c3_b_itmp;
  int32_T c3_b_k;
  int32_T c3_b_p1;
  int32_T c3_b_p2;
  int32_T c3_b_p3;
  int32_T c3_c_p2;
  int32_T c3_c_p3;
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
  boolean_T c3_x_dirtyOnGpu;
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
  cudaMemcpyToSymbol(c3_gpu_laneCoeffStds, chartInstance->c3_laneCoeffStds, 48UL,
                     0UL, cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(c3_gpu_laneCoeffMeans, chartInstance->c3_laneCoeffMeans,
                     48UL, 0UL, cudaMemcpyHostToDevice);
  c3_eML_blk_kernel_kernel3<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*chartInstance->c3_gpu_laneNetOut, *chartInstance->c3_b_gpu_params);
  if (!chartInstance->c3_vehicleXPoints_not_empty) {
    cudaMemcpyToSymbol(c3_gpu_vehicleXPoints, chartInstance->c3_vehicleXPoints,
                       224UL, 0UL, cudaMemcpyHostToDevice);
    c3_eML_blk_kernel_kernel4<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>();
    cudaMemcpyFromSymbol(chartInstance->c3_vehicleXPoints, c3_gpu_vehicleXPoints,
                         224UL, 0UL, cudaMemcpyDeviceToHost);
    chartInstance->c3_vehicleXPoints_not_empty = true;
  }

  cudaMemcpy(&c3_params[0], chartInstance->c3_b_gpu_params, 24UL,
             cudaMemcpyDeviceToHost);
  c3_params_dirtyOnGpu = false;
  if (muSingleScalarAbs(c3_params[5]) > 0.5F) {
    if (c3_params_dirtyOnGpu) {
      cudaMemcpy(&c3_params[0], chartInstance->c3_b_gpu_params, 24UL,
                 cudaMemcpyDeviceToHost);
    }

    if (muSingleScalarAbs(c3_params[2]) > 0.5F) {
      c3_eML_blk_kernel_kernel5<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_b_gpu_params, *chartInstance->c3_gpu_rt_y);
      for (c3_k = 0; c3_k < 2; c3_k++) {
        cudaMemcpyToSymbol(c3_gpu_vehicleXPoints,
                           chartInstance->c3_vehicleXPoints, 224UL, 0UL,
                           cudaMemcpyHostToDevice);
        c3_eML_blk_kernel_kernel6<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (c3_params[c3_k + 4], *chartInstance->c3_gpu_rt_y);
      }

      c3_eML_blk_kernel_kernel7<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_b_gpu_params, *chartInstance->c3_gpu_lt_y);
      for (c3_b_k = 0; c3_b_k < 2; c3_b_k++) {
        cudaMemcpyToSymbol(c3_gpu_vehicleXPoints,
                           chartInstance->c3_vehicleXPoints, 224UL, 0UL,
                           cudaMemcpyHostToDevice);
        c3_eML_blk_kernel_kernel8<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (c3_params[c3_b_k + 1], *chartInstance->c3_gpu_lt_y);
      }

      cudaMemcpy(chartInstance->c3_gpu_T, &c3_T[0], 72UL, cudaMemcpyHostToDevice);
      c3_eML_blk_kernel_kernel9<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_T, *chartInstance->c3_gpu_x);
      c3_p2 = 3;
      c3_p3 = 0;
      cudaMemcpy(&c3_x[0], chartInstance->c3_gpu_x, 72UL, cudaMemcpyDeviceToHost);
      c3_x_dirtyOnGpu = false;
      c3_z = c3_x[1] / 1594.5169999999998;
      if (c3_x_dirtyOnGpu) {
        cudaMemcpy(&c3_x[0], chartInstance->c3_gpu_x, 72UL,
                   cudaMemcpyDeviceToHost);
      }

      c3_x[1] /= 1594.5169999999998;
      c3_x[2] = 0.2;
      c3_x[4] -= c3_z * 2037.9982547799998;
      c3_x[5] = -150.06445095600003;
      c3_x[7] -= c3_z * 5.0;
      c3_x[8] = 0.0;
      if (150.06445095600003 > muDoubleScalarAbs(c3_x[4])) {
        c3_p2 = 0;
        c3_p3 = 3;
        c3_x[1] = 0.2;
        c3_x[2] = c3_z;
        c3_t1 = c3_x[4];
        c3_x[4] = -150.06445095600003;
        c3_x[5] = c3_t1;
        c3_t1 = c3_x[7];
        c3_x[7] = 0.0;
        c3_x[8] = c3_t1;
      }

      c3_b_z = c3_x[5] / c3_x[4];
      c3_x[5] /= c3_x[4];
      c3_x[8] -= c3_b_z * c3_x[7];
      c3_t3 = (c3_x[5] * c3_x[1] - c3_x[2]) / c3_x[8];
      c3_t2 = -(c3_x[1] + c3_x[7] * c3_t3) / c3_x[4];
      c3_Tinv[6] = ((1.0 - 2037.9982547799998 * c3_t2) - 5.0 * c3_t3) /
        1594.5169999999998;
      c3_Tinv[7] = c3_t2;
      c3_Tinv[8] = c3_t3;
      c3_t3 = -c3_x[5] / c3_x[8];
      c3_t2 = (1.0 - c3_x[7] * c3_t3) / c3_x[4];
      c3_Tinv[c3_p2] = -(2037.9982547799998 * c3_t2 + 5.0 * c3_t3) /
        1594.5169999999998;
      c3_Tinv[c3_p2 + 1] = c3_t2;
      c3_Tinv[c3_p2 + 2] = c3_t3;
      c3_t3 = 1.0 / c3_x[8];
      c3_t2 = -c3_x[7] * c3_t3 / c3_x[4];
      c3_Tinv[c3_p3] = -(2037.9982547799998 * c3_t2 + 5.0 * c3_t3) /
        1594.5169999999998;
      c3_Tinv[c3_p3 + 1] = c3_t2;
      c3_Tinv[c3_p3 + 2] = c3_t3;
      cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                 cudaMemcpyHostToDevice);
      c3_Tinv_dirtyOnCpu = false;
      c3_eML_blk_kernel_kernel10<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
      c3_p1 = 0;
      c3_b_p2 = 3;
      c3_b_p3 = 6;
      c3_absx11 = muDoubleScalarAbs(c3_Tinv[0]);
      c3_absx21 = muDoubleScalarAbs(c3_Tinv[1]);
      c3_absx31 = muDoubleScalarAbs(c3_Tinv[2]);
      if ((c3_absx21 > c3_absx11) && (c3_absx21 > c3_absx31)) {
        c3_p1 = 3;
        c3_b_p2 = 0;
        if (c3_Tinv_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                     cudaMemcpyHostToDevice);
          c3_Tinv_dirtyOnCpu = false;
        }

        c3_eML_blk_kernel_kernel12<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
      } else {
        if (c3_absx31 > c3_absx11) {
          c3_p1 = 6;
          c3_b_p3 = 0;
          if (c3_Tinv_dirtyOnCpu) {
            cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                       cudaMemcpyHostToDevice);
            c3_Tinv_dirtyOnCpu = false;
          }

          c3_eML_blk_kernel_kernel11<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
        }
      }

      cudaMemcpy(&c3_x[0], chartInstance->c3_gpu_x, 72UL, cudaMemcpyDeviceToHost);
      c3_c_z = c3_x[1] / c3_x[0];
      c3_x[1] /= c3_x[0];
      c3_d_z = c3_x[2] / c3_x[0];
      c3_x[2] /= c3_x[0];
      c3_x[4] -= c3_c_z * c3_x[3];
      c3_x[5] -= c3_d_z * c3_x[3];
      c3_x[7] -= c3_c_z * c3_x[6];
      c3_x[8] -= c3_d_z * c3_x[6];
      if (muDoubleScalarAbs(c3_x[5]) > muDoubleScalarAbs(c3_x[4])) {
        c3_itmp = c3_b_p2;
        c3_b_p2 = c3_b_p3;
        c3_b_p3 = c3_itmp;
        c3_x[1] = c3_d_z;
        c3_x[2] = c3_c_z;
        c3_b_t1 = c3_x[4];
        c3_x[4] = c3_x[5];
        c3_x[5] = c3_b_t1;
        c3_b_t1 = c3_x[7];
        c3_x[7] = c3_x[8];
        c3_x[8] = c3_b_t1;
      }

      c3_e_z = c3_x[5] / c3_x[4];
      c3_x[5] /= c3_x[4];
      c3_x[8] -= c3_e_z * c3_x[7];
      c3_b_t3 = (c3_x[5] * c3_x[1] - c3_x[2]) / c3_x[8];
      c3_b_t2 = -(c3_x[1] + c3_x[7] * c3_b_t3) / c3_x[4];
      cudaMemcpy(chartInstance->c3_gpu_x, &c3_x[0], 72UL, cudaMemcpyHostToDevice);
      c3_x_dirtyOnCpu = false;
      c3_eML_blk_kernel_kernel13<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (c3_b_t3, c3_b_t2, *chartInstance->c3_gpu_x, c3_p1,
         *chartInstance->c3_b_gpu_Tinv);
      c3_b_t3 = -c3_x[5] / c3_x[8];
      c3_b_t2 = (1.0 - c3_x[7] * c3_b_t3) / c3_x[4];
      if (c3_x_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c3_gpu_x, &c3_x[0], 72UL,
                   cudaMemcpyHostToDevice);
      }

      c3_eML_blk_kernel_kernel14<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (c3_b_t3, c3_b_t2, *chartInstance->c3_gpu_x, c3_b_p2,
         *chartInstance->c3_b_gpu_Tinv);
      c3_b_t3 = 1.0 / c3_x[8];
      c3_b_t2 = -c3_x[7] * c3_b_t3 / c3_x[4];
      c3_eML_blk_kernel_kernel15<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (c3_b_t3, c3_b_t2, *chartInstance->c3_gpu_x, c3_b_p3,
         *chartInstance->c3_b_gpu_Tinv);
      cudaMemcpyToSymbol(c3_gpu_vehicleXPoints, chartInstance->c3_vehicleXPoints,
                         224UL, 0UL, cudaMemcpyHostToDevice);
      c3_eML_blk_kernel_kernel16<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_lt_y, *chartInstance->c3_gpu_fv);
      c3_eML_blk_kernel_kernel17<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>
        (*chartInstance->c3_b_gpu_Tinv, *chartInstance->c3_gpu_fv,
         *chartInstance->c3_b_gpu_U);
      c3_eML_blk_kernel_kernel18<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_U, *chartInstance->c3_b_gpu_U,
         *chartInstance->c3_gpu_b);
      c3_eML_blk_kernel_kernel19<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
        (*chartInstance->c3_b_gpu_U, *chartInstance->c3_gpu_ltPts);
      c3_ltPts_dirtyOnGpu = true;
      if (c3_Tinv_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                   cudaMemcpyHostToDevice);
      }

      c3_eML_blk_kernel_kernel20<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
      c3_b_p1 = 0;
      c3_c_p2 = 3;
      c3_c_p3 = 6;
      c3_b_absx11 = muDoubleScalarAbs(c3_Tinv[0]);
      c3_b_absx21 = muDoubleScalarAbs(c3_Tinv[1]);
      c3_b_absx31 = muDoubleScalarAbs(c3_Tinv[2]);
      if ((c3_b_absx21 > c3_b_absx11) && (c3_b_absx21 > c3_b_absx31)) {
        c3_b_p1 = 3;
        c3_c_p2 = 0;
        c3_eML_blk_kernel_kernel22<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
      } else {
        if (c3_b_absx31 > c3_b_absx11) {
          c3_b_p1 = 6;
          c3_c_p3 = 0;
          c3_eML_blk_kernel_kernel21<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_x);
        }
      }

      cudaMemcpy(&c3_x[0], chartInstance->c3_gpu_x, 72UL, cudaMemcpyDeviceToHost);
      c3_f_z = c3_x[1] / c3_x[0];
      c3_x[1] /= c3_x[0];
      c3_g_z = c3_x[2] / c3_x[0];
      c3_x[2] /= c3_x[0];
      c3_x[4] -= c3_f_z * c3_x[3];
      c3_x[5] -= c3_g_z * c3_x[3];
      c3_x[7] -= c3_f_z * c3_x[6];
      c3_x[8] -= c3_g_z * c3_x[6];
      if (muDoubleScalarAbs(c3_x[5]) > muDoubleScalarAbs(c3_x[4])) {
        c3_b_itmp = c3_c_p2;
        c3_c_p2 = c3_c_p3;
        c3_c_p3 = c3_b_itmp;
        c3_x[1] = c3_g_z;
        c3_x[2] = c3_f_z;
        c3_c_t1 = c3_x[4];
        c3_x[4] = c3_x[5];
        c3_x[5] = c3_c_t1;
        c3_c_t1 = c3_x[7];
        c3_x[7] = c3_x[8];
        c3_x[8] = c3_c_t1;
      }

      c3_h_z = c3_x[5] / c3_x[4];
      c3_x[5] /= c3_x[4];
      c3_x[8] -= c3_h_z * c3_x[7];
      c3_c_t3 = (c3_x[5] * c3_x[1] - c3_x[2]) / c3_x[8];
      c3_c_t2 = -(c3_x[1] + c3_x[7] * c3_c_t3) / c3_x[4];
      c3_Tinv[c3_b_p1] = ((1.0 - c3_x[3] * c3_c_t2) - c3_x[6] * c3_c_t3) / c3_x
        [0];
      c3_Tinv[c3_b_p1 + 1] = c3_c_t2;
      c3_Tinv[c3_b_p1 + 2] = c3_c_t3;
      c3_c_t3 = -c3_x[5] / c3_x[8];
      c3_c_t2 = (1.0 - c3_x[7] * c3_c_t3) / c3_x[4];
      c3_Tinv[c3_c_p2] = -(c3_x[3] * c3_c_t2 + c3_x[6] * c3_c_t3) / c3_x[0];
      c3_Tinv[c3_c_p2 + 1] = c3_c_t2;
      c3_Tinv[c3_c_p2 + 2] = c3_c_t3;
      c3_c_t3 = 1.0 / c3_x[8];
      c3_c_t2 = -c3_x[7] * c3_c_t3 / c3_x[4];
      c3_Tinv[c3_c_p3] = -(c3_x[3] * c3_c_t2 + c3_x[6] * c3_c_t3) / c3_x[0];
      cudaMemcpy(chartInstance->c3_gpu_Tinv, &c3_Tinv[0], 72UL,
                 cudaMemcpyHostToDevice);
      c3_eML_blk_kernel_kernel23<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (c3_c_t3, c3_c_t2, c3_c_p3, *chartInstance->c3_gpu_Tinv);
      cudaMemcpyToSymbol(c3_gpu_vehicleXPoints, chartInstance->c3_vehicleXPoints,
                         224UL, 0UL, cudaMemcpyHostToDevice);
      c3_eML_blk_kernel_kernel24<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_rt_y, *chartInstance->c3_gpu_fv1);
      c3_eML_blk_kernel_kernel25<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>
        (*chartInstance->c3_gpu_Tinv, *chartInstance->c3_gpu_fv1,
         *chartInstance->c3_b_gpu_U);
      c3_eML_blk_kernel_kernel26<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
        (*chartInstance->c3_c_gpu_U, *chartInstance->c3_b_gpu_U,
         *chartInstance->c3_gpu_b);
      c3_eML_blk_kernel_kernel27<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
        (*chartInstance->c3_b_gpu_U, *chartInstance->c3_gpu_rtPts);
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
  real_T c3_T[9], real_T c3_x[9])
{
  int32_T c3_i12;
  c3_i12 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i12 < 9) {
    c3_x[c3_i12] = c3_T[c3_i12];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel10(const
  real_T c3_Tinv[9], real_T c3_x[9])
{
  int32_T c3_i13;
  c3_i13 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i13 < 9) {
    c3_x[c3_i13] = c3_Tinv[c3_i13];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel11(const
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

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel12(const
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

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel13(const
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

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel14(const
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

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel15(const
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

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel16(const
  real32_T c3_lt_y[28], real32_T c3_fv[84])
{
  int32_T c3_i14;
  c3_i14 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i14 < 28) {
    c3_fv[c3_i14] = (real32_T)c3_gpu_vehicleXPoints[c3_i14];
    c3_fv[c3_i14 + 28] = c3_lt_y[c3_i14];
    c3_fv[c3_i14 + 56] = 1.0F;
  }
}

static __global__ __launch_bounds__(96, 1) void c3_eML_blk_kernel_kernel17(const
  real_T c3_Tinv[9], const real32_T c3_fv[84], real32_T c3_U[84])
{
  uint64_T c3_threadId;
  int32_T c3_i15;
  int32_T c3_i16;
  int32_T c3_i17;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_i17 = (int32_T)(c3_threadId % 3UL);
  c3_i15 = (int32_T)((c3_threadId - (uint64_T)c3_i17) / 3UL);
  if ((c3_i15 < 28) && (c3_i17 < 3)) {
    c3_U[c3_i15 + 28 * c3_i17] = 0.0F;
    for (c3_i16 = 0; c3_i16 < 3; c3_i16++) {
      c3_U[c3_i15 + 28 * c3_i17] += c3_fv[c3_i15 + 28 * c3_i16] * (real32_T)
        c3_Tinv[c3_i16 + 3 * c3_i17];
    }
  }
}

static __global__ __launch_bounds__(64, 1) void c3_eML_blk_kernel_kernel18
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

static __global__ __launch_bounds__(64, 1) void c3_eML_blk_kernel_kernel19(const
  real32_T c3_U[84], real32_T c3_b_ltPts[56])
{
  uint64_T c3_threadId;
  int32_T c3_i18;
  int32_T c3_i20;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_i20 = (int32_T)(c3_threadId % 28UL);
  c3_i18 = (int32_T)((c3_threadId - (uint64_T)c3_i20) / 28UL);
  if ((c3_i18 < 2) && (c3_i20 < 28)) {
    c3_b_ltPts[c3_i20 + 28 * c3_i18] = c3_U[c3_i20 + 28 * c3_i18];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel20(const
  real_T c3_Tinv[9], real_T c3_x[9])
{
  int32_T c3_i19;
  c3_i19 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i19 < 9) {
    c3_x[c3_i19] = c3_Tinv[c3_i19];
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel21(const
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

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel22(const
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

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel23(const
  real_T c3_t3, const real_T c3_t2, const int32_T c3_p3, real_T c3_Tinv[9])
{
  int32_T c3_tmpIdx;
  c3_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c3_tmpIdx < 1) {
    c3_Tinv[c3_p3 + 1] = c3_t2;
    c3_Tinv[c3_p3 + 2] = c3_t3;
  }
}

static __global__ __launch_bounds__(32, 1) void c3_eML_blk_kernel_kernel24(const
  real32_T c3_rt_y[28], real32_T c3_fv1[84])
{
  int32_T c3_i21;
  c3_i21 = (int32_T)mwGetGlobalThreadIndex();
  if (c3_i21 < 28) {
    c3_fv1[c3_i21] = (real32_T)c3_gpu_vehicleXPoints[c3_i21];
    c3_fv1[c3_i21 + 28] = c3_rt_y[c3_i21];
    c3_fv1[c3_i21 + 56] = 1.0F;
  }
}

static __global__ __launch_bounds__(96, 1) void c3_eML_blk_kernel_kernel25(const
  real_T c3_Tinv[9], const real32_T c3_fv1[84], real32_T c3_U[84])
{
  uint64_T c3_threadId;
  int32_T c3_i22;
  int32_T c3_i23;
  int32_T c3_i24;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_i24 = (int32_T)(c3_threadId % 3UL);
  c3_i22 = (int32_T)((c3_threadId - (uint64_T)c3_i24) / 3UL);
  if ((c3_i22 < 28) && (c3_i24 < 3)) {
    c3_U[c3_i22 + 28 * c3_i24] = 0.0F;
    for (c3_i23 = 0; c3_i23 < 3; c3_i23++) {
      c3_U[c3_i22 + 28 * c3_i24] += c3_fv1[c3_i22 + 28 * c3_i23] * (real32_T)
        c3_Tinv[c3_i23 + 3 * c3_i24];
    }
  }
}

static __global__ __launch_bounds__(64, 1) void c3_eML_blk_kernel_kernel26
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

static __global__ __launch_bounds__(64, 1) void c3_eML_blk_kernel_kernel27(const
  real32_T c3_U[84], real32_T c3_b_rtPts[56])
{
  uint64_T c3_threadId;
  int32_T c3_i25;
  int32_T c3_i26;
  c3_threadId = mwGetGlobalThreadIndex();
  c3_i26 = (int32_T)(c3_threadId % 28UL);
  c3_i25 = (int32_T)((c3_threadId - (uint64_T)c3_i26) / 28UL);
  if ((c3_i25 < 2) && (c3_i26 < 28)) {
    c3_b_rtPts[c3_i26 + 28 * c3_i25] = c3_U[c3_i26 + 28 * c3_i25];
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
  const char* encStrCodegen [25] = {
    "eNrtWM1v40QUd0t3oYhWFVoJtFqJnoDTph+7qCdImw81Uj+iTdtFXLJT+zkeMp5xZ8ZJizjBAY7",
    "8CRw5cuA/4MYFcYP/ghMnDrxx3DR17KRpS5pALaXu2L/3/Hvv/ebN2NZMZdfCYxF/y+9b1kM8v4",
    "G/WatzPIjHMz2/zvU564N4/BMa8dCvEkl8ZQ08OPHhBSjBQk0Fr3BXpMIod0ECtxEbCKmzvCnqh",
    "4zyZjnktvGnXnrU9mqeCJmzhbbE2efsDL0Foa6inyKVYOsygKM9KcKGV2ak0WUsdbvggd1UoT8o",
    "BAW6FgaGltoNmaYBg9Ip2BWuNEHG6oJbTRMNBX2aGaaJVNXOgcIPGCU8NVqPqBoEmGANh4GDf/d",
    "DjUElYbZHpN4Cj7RA7dBm5FNwSPqkCm8cU060kJSwks8KxrCfW5Uhn13hABuQEOS2JYE0A0G5zq",
    "5/rYyRljg5ZlCE47CR7a0GJ6Ep/hGFNsjMvLkF0QJJGrDPMx8aJaR0GlWrq5J+mKY+HBG5aWP9F",
    "DiZ6kXlqBrBOsEBWmTBIAqyog4kbWF6M72FfsUoc9iUCf1OsdUwWOSt1IJBVeh6K9u8QBhTmbAD",
    "EexAC1jktUg0GQzreE3HKUWdA4EJNvLOng0hp1j4GFYQ3KGp5WolAFHf2cPGchlph0oLv4DiLe7",
    "s9N/uh1W4BukSG9K6gCRUAeYsSm+2N4cqU3sEIisd0UsDdxQyDGUpN+TFtpBNzMmAJnIRgqloJt",
    "BXDawlzoRDhZNmEMzUchjOJrYHjmkwlMEuThvEpuREmda2ifOuRfVZEZQtaZBS1RBnHbahkhHUW",
    "QCHvMlFm5el8Gtxj+9UATsD9nAfa3AQzTFuoyuqNLYLevF4BwBFSSSnvLGFbU6elZFkasXMurdi",
    "Xax7b11h3Tu3S54/7PEzk+LH6jknnzs/O/i5s/jfTGyX77FbSDxnLmFncEvmWX/88tejX7e//Pb",
    "n539/89uLjZs8/9Wbo+0TFuPxk/OG3J1grT5dG+x2D6+5FP/v9PhfisfK3tiofUG3258d1w5bpe",
    "rLvRX/qFqK/P04P5jv6wm+59eXzcqAaox0LO2KE29gzJiEnWXd+N/o4ftwSD7m4+ud489Pbmb/K",
    "J/UQ1q+5hP5MmNc2KEsQu6k6Pju4nk3n7RPi+dBIh4zZroarXqTEceTa8chbzWOpfzN7Pv7TVoc",
    "i4k4FmN9FQS47i4Q0+wjHjOj9Y3ZeGxsmEWtIIVPWjyziXiWutHU89EpT/K3oQ9vSF5WE3kx45w",
    "nfMh5vs1ym0VcvLQULGeSVXdAQ7Q/rdtCSMfszkE99adIBwuJeBd6dVDTzrmu71wH3wUdHbw6ud",
    "fBmPpBC/DVnMGn1fglcTJ08FVHB9bX/zMdLN+KDqpD4n2ciPdx9N5eJ+ZtAOr2en0H4yyeh9m/D",
    "7nu/nRUO+ve7t5uDHYz13zfvK7dazd8vx2X3U3jG/W9e9LwKwP6qJXAL01wHDf9HvJv43+3Rttv",
    "vBePP+5+wix4lDkpX7Pi2ztA3LS7/xGdmkk5CP92Av/9kP2BSOhaZO2HFAvk57m6cs3p0qYBLwJ",
    "zc+ZDaU5JO2dzXteSUA6Owe2BXrm8y1ipu7a5sFZvPz2mfAp4Hl+Zp5/g6Y+FJzptrcfZtCac52",
    "qczUnnuTYlPFdHqnuQ4BmMhadDNKkrmzAwVCc7n8+mpO7rU8Lz+ZT0pbWReN7lerQ6wrp5l3WfD",
    "n0+u1T3H4bwPEnwPBlf/xSuq0DHXK+6v2sm+DbHpNOPutWfdJ7tkXje7bzv5PQfX5E00Q==",
    ""
  };

  static char newstr [1797] = "";
  newstr[0] = '\0';
  for (i = 0; i < 25; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c3_LaneDetection(SimStruct *S)
{
  const char* newstr = sf_c3_LaneDetection_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(4140489008U));
  ssSetChecksum1(S,(2085146388U));
  ssSetChecksum2(S,(4231381896U));
  ssSetChecksum3(S,(944951687U));
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
