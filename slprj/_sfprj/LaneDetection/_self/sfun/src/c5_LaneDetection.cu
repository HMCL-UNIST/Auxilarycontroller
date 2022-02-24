/* Include files */

#include "LaneDetection_sfun.h"
#include "c5_LaneDetection.h"
#include <string.h>
#include "MWCudaDimUtility.hpp"
#include "MWLaunchParametersUtilities.hpp"
#include "MWShuffleUtility.h"
#include "MWSortFunctors.h"
#include "MWSortWithIndexUtility.h"
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

#include <cstdlib>
#include <cstring>

/* Type Definitions */

/* Named Constants */
const int32_T CALL_EVENT = -1;

/* Variable Declarations */

/* Variable Definitions */
static real_T _sfTime_;

/* Function Declarations */
static void initialize_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static void initialize_params_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct *
  chartInstance);
static void enable_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static void disable_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static void c5_do_animation_call_c5_LaneDetection
  (SFc5_LaneDetectionInstanceStruct *chartInstance);
static void ext_mode_exec_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c5_LaneDetection
  (SFc5_LaneDetectionInstanceStruct *chartInstance);
static void set_sim_state_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_st);
static void sf_gateway_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_start_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_terminate_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_setup_runtime_resources_c5_LaneDetection
  (SFc5_LaneDetectionInstanceStruct *chartInstance);
static void mdl_cleanup_runtime_resources_c5_LaneDetection
  (SFc5_LaneDetectionInstanceStruct *chartInstance);
static void initSimStructsc5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static void c5_eML_blk_kernel(SFc5_LaneDetectionInstanceStruct *chartInstance,
  real32_T c5_b_In[921600], real_T c5_b_bboxes_data[], int32_T c5_bboxes_size[2],
  real32_T c5_b_scores_data[], int32_T c5_scores_size[2]);
static void c5_DeepLearningNetwork_setup(SFc5_LaneDetectionInstanceStruct
  *chartInstance, c5_yolov2ResNet50VehicleExample0_LaneDetection0 *c5_obj);
static real32_T c5_callFcn(SFc5_LaneDetectionInstanceStruct *chartInstance,
  real32_T c5_input1, real32_T c5_input2);
static real32_T c5_b_callFcn(SFc5_LaneDetectionInstanceStruct *chartInstance,
  real32_T c5_input1, real32_T c5_input2);
static void c5_DeepLearningNetwork_activations(SFc5_LaneDetectionInstanceStruct *
  chartInstance, c5_yolov2ResNet50VehicleExample0_LaneDetection0 *c5_obj,
  real32_T c5_varargin_1[150528], real32_T c5_b_out[4704]);
static void c5_indexShapeCheck(SFc5_LaneDetectionInstanceStruct *chartInstance,
  int32_T c5_matrixSize, int32_T c5_indexSize[2]);
static void c5_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct *chartInstance,
  const mxArray *c5_bboxes, const char_T *c5_identifier, real_T c5_y_data[],
  int32_T c5_y_size[2]);
static void c5_b_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y_data[], int32_T c5_y_size[2]);
static void c5_c_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_scores, const char_T *c5_identifier,
  real32_T c5_y_data[], int32_T c5_y_size[2]);
static void c5_d_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real32_T c5_y_data[], int32_T c5_y_size[2]);
static uint8_T c5_e_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_LaneDetection, const char_T
  *c5_identifier);
static uint8_T c5_f_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId);
static void c5_emxConvertDynamicMatrixFromEmx_(SFc5_LaneDetectionInstanceStruct *
  chartInstance, c5_emxArray_real_T_20x4 *c5_emx, real_T c5_data[80], int32_T
  c5_size[2]);
static void c5_b_emxConvertDynamicMatrixFromEmx_
  (SFc5_LaneDetectionInstanceStruct *chartInstance, c5_emxArray_real32_T_20x1
   *c5_emx, real32_T c5_data[20], int32_T c5_size[2]);
static __global__ void c5_coder_reduce0(const real32_T c5_inputVar[150528],
  real32_T *c5_outputVar);
static __device__ real32_T c5_threadGroupReduction(real32_T c5_val, uint32_T
  c5_lane, uint32_T c5_mask);
static __device__ real32_T c5_shflDown1(real32_T c5_in1, uint32_T c5_offset,
  uint32_T c5_mask);
static __device__ real32_T c5_workGroupReduction(real32_T c5_val, uint32_T
  c5_mask, uint32_T c5_numActiveWarps);
static __device__ real32_T c5_b_threadGroupReduction(real32_T c5_val, uint32_T
  c5_lane, uint32_T c5_mask);
static __device__ real32_T c5_b_workGroupReduction(real32_T c5_val, uint32_T
  c5_mask, uint32_T c5_numActiveWarps);
static __device__ real32_T c5_atomicOpreal32_T(real32_T *c5_address, real32_T
  c5_value);
static __device__ real32_T c5_b_atomicOpreal32_T(real32_T *c5_address, real32_T
  c5_value);
static __global__ void c5_eML_blk_kernel_kernel1(int16_T c5_aux1[960]);
static __global__ void c5_eML_blk_kernel_kernel2(int16_T c5_aux2[1280]);
static __global__ void c5_eML_blk_kernel_kernel3(const int16_T c5_aux1[960],
  real_T c5_rowWeights[2016], int16_T c5_ipRowIndices[2016]);
static __global__ void c5_eML_blk_kernel_kernel4(const int16_T c5_aux2[1280],
  real_T c5_colWeights[2688], int16_T c5_ipColIndices[2688]);
static __global__ void c5_eML_blk_kernel_kernel5(const real_T c5_rowWeights[2016],
  real_T c5_rowWeightsTotal[224]);
static __global__ void c5_eML_blk_kernel_kernel6(const real_T c5_rowWeights[2016],
  const int32_T c5_xoffset, real_T c5_rowWeightsTotal[224]);
static __global__ void c5_eML_blk_kernel_kernel7(const real_T c5_colWeights[2688],
  real_T c5_colWeightsTotal[224]);
static __global__ void c5_eML_blk_kernel_kernel8(const real_T c5_colWeights[2688],
  const int32_T c5_xoffset, real_T c5_colWeightsTotal[224]);
static __global__ void c5_eML_blk_kernel_kernel9(const real_T
  c5_colWeightsTotal[224], const real_T c5_colWeights[2688], const real32_T
  c5_b_In[921600], const int16_T c5_ipColIndices[2688], real32_T
  c5_partialResize[322560]);
static __global__ void c5_eML_blk_kernel_kernel10(const real_T
  c5_rowWeightsTotal[224], const real_T c5_rowWeights[2016], const real32_T
  c5_partialResize[322560], const int16_T c5_ipRowIndices[2016], real32_T
  c5_b_out[150528]);
static __global__ void c5_eML_blk_kernel_kernel11(real32_T c5_b_out[150528],
  real32_T c5_outVal[2]);
static __global__ void c5_eML_blk_kernel_kernel12(const real32_T c5_y, const
  real32_T c5_outVal, real32_T c5_b_out[150528], real32_T c5_c_out[150528]);
static __global__ void c5_eML_blk_kernel_kernel13(const int8_T c5_dv[8], real_T
  c5_anchors[8]);
static __global__ void c5_eML_blk_kernel_kernel14(const real_T c5_dv1[4], real_T
  c5_anchors[4], real_T c5_b_anchors[8]);
static __global__ void c5_eML_blk_kernel_kernel15(const real_T c5_anchors[8],
  const real32_T c5_tmpFeatureMap[4704], real32_T c5_boxOut[4704]);
static __global__ void c5_eML_blk_kernel_kernel16(const real32_T c5_boxOut[4704],
  boolean_T c5_bv[784]);
static __global__ void c5_eML_blk_kernel_kernel17(const real32_T c5_boxOut[4704],
  const int16_T c5_ii_data[784], const int32_T c5_thresholdedPrediction_size[2],
  const int32_T c5_ii_size[1], real32_T c5_thresholdedPrediction_data[4704]);
static __global__ void c5_eML_blk_kernel_kernel18(const real32_T
  c5_thresholdedPrediction_data[4704], const int32_T
  c5_thresholdedPrediction_size[2], const int32_T c5_bboxesX1Y1X2Y2_size[2],
  const int32_T c5_i7, real_T c5_bboxesX1Y1X2Y2_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel19(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_i10, real_T c5_x1_data[784]);
static __global__ void c5_eML_blk_kernel_kernel20(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_i14, real_T c5_y1_data[784]);
static __global__ void c5_eML_blk_kernel_kernel21(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_i17, real_T c5_x2_data[784]);
static __global__ void c5_eML_blk_kernel_kernel22(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_i19, real_T c5_y2_data[784]);
static __global__ void c5_eML_blk_kernel_kernel23(const int32_T c5_end, real_T
  c5_x1_data[784]);
static __global__ void c5_eML_blk_kernel_kernel24(const int32_T c5_end, real_T
  c5_y1_data[784]);
static __global__ void c5_eML_blk_kernel_kernel25(const int32_T c5_end, real_T
  c5_x2_data[784]);
static __global__ void c5_eML_blk_kernel_kernel26(const int32_T c5_end, real_T
  c5_y2_data[784]);
static __global__ void c5_eML_blk_kernel_kernel27(const real_T c5_x1_data[784],
  const int32_T c5_loop_ub, real_T c5_bboxesX1Y1X2Y2_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel28(const real_T c5_y1_data[784],
  const int32_T c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_loop_ub, real_T
  c5_bboxesX1Y1X2Y2_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel29(const real_T c5_x2_data[784],
  const int32_T c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_loop_ub, real_T
  c5_bboxesX1Y1X2Y2_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel30(const real_T c5_y2_data[784],
  const int32_T c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_loop_ub, real_T
  c5_bboxesX1Y1X2Y2_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel31(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_loop_ub, real_T
  c5_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel32(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_bboxPred_size[2], const int32_T c5_loop_ub, real_T
  c5_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel33(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_bboxPred_size[2], const int32_T c5_loop_ub, real_T
  c5_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel34(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_bboxPred_size[2], const int32_T c5_loop_ub, real_T
  c5_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel35(const int32_T c5_nx, real_T
  c5_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel36(const real_T c5_bboxPred_data
  [3136], const int32_T c5_bboxPred_size[2], const int32_T c5_i32, real_T
  c5_b_bboxPred_data[784]);
static __global__ void c5_eML_blk_kernel_kernel37(const real_T c5_bboxPred_data
  [784], const int32_T c5_bboxPred_size[2], const int32_T c5_b_bboxPred_size[1],
  real_T c5_b_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel38(const real_T c5_bboxPred_data
  [3136], const int32_T c5_bboxPred_size[2], const int32_T c5_i35, real_T
  c5_b_bboxPred_data[784]);
static __global__ void c5_eML_blk_kernel_kernel39(const real_T c5_bboxPred_data
  [784], const int32_T c5_bboxPred_size[2], const int32_T c5_b_bboxPred_size[1],
  real_T c5_b_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel40(const real_T c5_bboxPred_data
  [3136], const int32_T c5_bboxPred_size[2], const int32_T c5_i, const int32_T
  c5_b_bboxPred_size[2], const int32_T c5_count, real_T c5_b_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel41(const real32_T
  c5_thresholdedPrediction_data[4704], const int32_T
  c5_thresholdedPrediction_size[2], const int32_T c5_i, const real_T c5_count,
  real32_T c5_classPred_data[784], real32_T c5_scorePred_data[784]);
static __global__ void c5_eML_blk_kernel_kernel42(const int32_T c5_i38, const
  int32_T c5_i39, int32_T c5_idx_data[784]);
static __global__ void c5_eML_blk_kernel_kernel43(const int32_T
  c5_bboxPred_size[2], boolean_T c5_b_data[784]);
static __global__ void c5_eML_blk_kernel_kernel44(const int32_T c5_b_size[2],
  const boolean_T c5_b_data[784], int32_T c5_i3, int32_T *c5_n);
static __device__ int32_T c5_c_threadGroupReduction(int32_T c5_val, uint32_T
  c5_lane, uint32_T c5_mask);
static __device__ int32_T c5_b_shflDown1(int32_T c5_in1, uint32_T c5_offset,
  uint32_T c5_mask);
static __device__ int32_T c5_c_workGroupReduction(int32_T c5_val, uint32_T
  c5_mask, uint32_T c5_numActiveWarps);
static __global__ void c5_eML_blk_kernel_kernel45(const int32_T *c5_n, const
  int32_T c5_bboxPred_size[2], int32_T *c5_nrows);
static __global__ void c5_eML_blk_kernel_kernel46(const int32_T
  c5_bboxPred_size[2], const int32_T *c5_nrows, const int32_T c5_idx_data[784],
  real_T c5_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel47(const real_T c5_bboxPred_data
  [3136], const int32_T c5_bboxPred_size[2], const int32_T c5_b_bboxPred_size[2],
  const int32_T c5_i4, real_T c5_b_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel48(const real_T c5_bboxPred_data
  [3136], const int32_T c5_bboxPred_size[2], real_T c5_b_bboxPred_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel49(const int32_T c5_i45, const
  int32_T c5_i47, int32_T c5_idx_data[784]);
static __global__ void c5_eML_blk_kernel_kernel50(const int32_T
  c5_scorePred_size[1], boolean_T c5_b_data[784]);
static __global__ void c5_eML_blk_kernel_kernel51(const int32_T c5_b_size[2],
  const boolean_T c5_b_data[784], int32_T c5_i6, int32_T *c5_n);
static __global__ void c5_eML_blk_kernel_kernel52(const int32_T c5_i50, const
  int32_T c5_i51, int32_T c5_idx_data[784]);
static __global__ void c5_eML_blk_kernel_kernel53(const int32_T
  c5_classPred_size[1], boolean_T c5_b_data[784]);
static __global__ void c5_eML_blk_kernel_kernel54(const int32_T c5_b_size[2],
  const boolean_T c5_b_data[784], int32_T c5_i9, int32_T *c5_n);
static __global__ void c5_eML_blk_kernel_kernel55(const real32_T
  c5_scorePred_data[784], const int32_T c5_scorePred_size[1], real32_T
  c5_out_data[784]);
static __global__ void c5_eML_blk_kernel_kernel56(const uint32_T c5_dv2[2],
  real_T c5_x1_data[784]);
static __global__ void c5_eML_blk_kernel_kernel57(const real_T c5_bboxPred_data
  [3136], const int32_T c5_bboxPred_size[2], const real_T c5_x1_data[784], const
  int32_T c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_x1_size[1], real_T
  c5_bboxesX1Y1X2Y2_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel58(const real32_T
  c5_classPred_data[784], const real_T c5_x1_data[784], const int32_T
  c5_x1_size[1], real_T c5_y1_data[784]);
static __global__ void c5_eML_blk_kernel_kernel59(const int32_T c5_x1_size[1],
  boolean_T c5_selectedIndex_data[784]);
static __global__ void c5_eML_blk_kernel_kernel60(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_i61, real_T c5_area_data[784]);
static __global__ void c5_eML_blk_kernel_kernel61(const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const real_T c5_bboxesX1Y1X2Y2_data[3136], const
  int32_T c5_i64, real_T c5_x2_data[784]);
static __global__ void c5_eML_blk_kernel_kernel62(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_i66, real_T c5_y2_data[784]);
static __global__ void c5_eML_blk_kernel_kernel63(const int32_T c5_i12, const
  int32_T c5_iv[2], boolean_T c5_selectedIndex_data[784]);
static __global__ void c5_eML_blk_kernel_kernel64(const boolean_T
  c5_selectedIndex_data[784], const real_T c5_x1_data[784], const int32_T
  c5_selectedIndex_size[1], boolean_T c5_index_data[784]);
static __global__ void c5_eML_blk_kernel_kernel65(const real_T c5_bboxPred_data
  [3136], const int32_T c5_bboxPred_size[2], const int16_T c5_iv1_data[784],
  const int32_T c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_iv1_size[1], real_T
  c5_bboxesX1Y1X2Y2_data[3136]);
static __global__ void c5_eML_blk_kernel_kernel66(const real32_T
  c5_scorePred_data[784], const int32_T c5_scorePred_size[1], real32_T
  c5_b_scores_data[784]);
static __global__ void c5_eML_blk_kernel_kernel67(const real_T
  c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_bboxesX1Y1X2Y2_size[2], real_T
  c5_b_bboxes_data[]);
static __global__ void c5_eML_blk_kernel_kernel68(const real32_T
  c5_b_scores_data[784], const int32_T c5_scores_size[1], real32_T
  c5_c_scores_data[]);
static __global__ void c5_DeepLearningNetwork_activations_kernel69(const
  real32_T c5_varargin_1[150528], c5_cell_wrap_18 *c5_r);
static __global__ void c5_DeepLearningNetwork_activations_kernel70(const
  c5_cell_wrap_18 *c5_r, c5_cell_wrap_18 c5_miniBatchT[1]);
static __global__ void c5_DeepLearningNetwork_activations_kernel71(const
  real32_T c5_outMiniBatch[4704], real32_T c5_b_out[4704]);
static __device__ real32_T c5_callFcn_device(real32_T c5_input1, real32_T
  c5_input2);
static __device__ real32_T c5_b_callFcn_device(real32_T c5_input1, real32_T
  c5_input2);
static void c5_checkCleanupCudaError(cudaError_t c5_errCode, const char_T
  *c5_file, uint32_T c5_line);
static emlrtRTEInfo c5_createEmlrtInfoStruct(const char_T *c5_file, uint32_T
  c5_line);
static void init_dsm_address_info(SFc5_LaneDetectionInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc5_LaneDetectionInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
  emlrtLicenseCheckR2012b(chartInstance->c5_fEmlrtCtx,
    "distrib_computing_toolbox", 2);
  emlrtLicenseCheckR2012b(chartInstance->c5_fEmlrtCtx, "neural_network_toolbox",
    2);
  emlrtLicenseCheckR2012b(chartInstance->c5_fEmlrtCtx,
    "video_and_image_blockset", 2);
  sim_mode_is_external(chartInstance->S);
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c5_yolodetector_not_empty = false;
  chartInstance->c5_is_active_c5_LaneDetection = 0U;
  cudaGetLastError();
  cudaMalloc(&chartInstance->c5_d_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i7, 4UL);
  cudaMalloc(&chartInstance->c5_c_gpu_bboxPred_size, 4UL);
  cudaMalloc(&chartInstance->c5_d_gpu_end, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_aux2, 2560UL);
  cudaMalloc(&chartInstance->c5_gpu_classPred_size, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_xoffset, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxPred_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_end, 4UL);
  cudaMalloc(&chartInstance->c5_d_gpu_bboxPred_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_i47, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_rowWeights, 16128UL);
  cudaMalloc(&chartInstance->c5_gpu_i64, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxesX1Y1X2Y2_data, 25088UL);
  cudaMalloc(&chartInstance->c5_gpu_idx_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_i9, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_scores_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_bv, 784UL);
  cudaMalloc(&chartInstance->c5_gpu_selectedIndex_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_outVal, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_selectedIndex_data, 784UL);
  cudaMalloc(&chartInstance->c5_c_gpu_end, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_boxOut, 18816UL);
  cudaMalloc(&chartInstance->c5_h_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i35, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_scores_data, 20U * sizeof(real32_T));
  cudaMalloc(&chartInstance->c5_gpu_count, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i61, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_b_size, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_i10, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_end, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_anchors, 32UL);
  cudaMalloc(&chartInstance->c5_gpu_i, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_ii_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_colWeightsTotal, 1792UL);
  cudaMalloc(&chartInstance->c5_c_gpu_n, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i51, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_count, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_nx, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i50, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_scores_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_iv, 8UL);
  cudaMalloc(&chartInstance->c5_c_gpu_bboxPred_data, 25088UL);
  cudaMalloc(&chartInstance->c5_gpu_index_data, 784UL);
  cudaMalloc(&chartInstance->c5_gpu_b_data, 784UL);
  cudaMalloc(&chartInstance->c5_gpu_In, 3686400UL);
  cudaMalloc(&chartInstance->c5_b_gpu_outVal, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_dv, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_area_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_nrows, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxes_data, 80U * sizeof(real_T));
  cudaMalloc(&chartInstance->c5_gpu_y1_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_i17, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_iv1_data, 1568UL);
  cudaMalloc(&chartInstance->c5_gpu_i39, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_x2_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_i38, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_out, 602112UL);
  cudaMalloc(&chartInstance->c5_gpu_classPred_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_rowWeightsTotal, 1792UL);
  cudaMalloc(&chartInstance->c5_gpu_i32, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_dv2, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_ipRowIndices, 4032UL);
  cudaMalloc(&chartInstance->c5_gpu_i14, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i66, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_x1_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_colWeights, 21504UL);
  cudaMalloc(&chartInstance->c5_b_gpu_bboxPred_data, 25088UL);
  cudaMalloc(&chartInstance->c5_b_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_e_gpu_bboxPred_size, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_anchors, 64UL);
  cudaMalloc(&chartInstance->c5_gpu_y, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_scorePred_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_ii_data, 1568UL);
  cudaMalloc(&chartInstance->c5_gpu_thresholdedPrediction_data, 18816UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxPred_size, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_i19, 4UL);
  cudaMalloc(&chartInstance->c5_d_gpu_bboxPred_size, 8UL);
  cudaMalloc(&chartInstance->c5_g_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_partialResize, 1290240UL);
  cudaMalloc(&chartInstance->c5_b_gpu_n, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_xoffset, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i45, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_thresholdedPrediction_size, 8UL);
  cudaMalloc(&chartInstance->c5_e_gpu_bboxPred_data, 25088UL);
  cudaMalloc(&chartInstance->c5_gpu_aux1, 1920UL);
  cudaMalloc(&chartInstance->c5_gpu_x1_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_dv1, 32UL);
  cudaMalloc(&chartInstance->c5_gpu_out_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_scorePred_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i6, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i3, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_n, 4UL);
  cudaMalloc(&chartInstance->c5_f_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_ipColIndices, 5376UL);
  cudaMalloc(&chartInstance->c5_c_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_bboxPred_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_y2_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_i12, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_out, 602112UL);
  cudaMalloc(&chartInstance->c5_gpu_iv1_size, 4UL);
  cudaMalloc(&chartInstance->c5_e_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i4, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_tmpFeatureMap, 18816UL);
}

static void initialize_params_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct *
  chartInstance)
{
}

static void enable_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c5_do_animation_call_c5_LaneDetection
  (SFc5_LaneDetectionInstanceStruct *chartInstance)
{
  sfDoAnimationWrapper(chartInstance->S, false, true);
  sfDoAnimationWrapper(chartInstance->S, false, false);
}

static void ext_mode_exec_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c5_LaneDetection
  (SFc5_LaneDetectionInstanceStruct *chartInstance)
{
  const mxArray *c5_b_y = NULL;
  const mxArray *c5_c_y = NULL;
  const mxArray *c5_d_y = NULL;
  const mxArray *c5_st;
  const mxArray *c5_y = NULL;
  c5_st = NULL;
  c5_y = NULL;
  sf_mex_assign(&c5_y, sf_mex_createcellmatrix(3, 1), false);
  c5_b_y = NULL;
  sf_mex_assign(&c5_b_y, sf_mex_create("y", chartInstance->c5_bboxes_data, 0, 0U,
    1U, 0U, 2, (*chartInstance->c5_bboxes_sizes)[0],
    (*chartInstance->c5_bboxes_sizes)[1]), false);
  sf_mex_setcell(c5_y, 0, c5_b_y);
  c5_c_y = NULL;
  sf_mex_assign(&c5_c_y, sf_mex_create("y", chartInstance->c5_scores_data, 1, 0U,
    1U, 0U, 2, (*chartInstance->c5_scores_sizes)[0], 1), false);
  sf_mex_setcell(c5_y, 1, c5_c_y);
  c5_d_y = NULL;
  sf_mex_assign(&c5_d_y, sf_mex_create("y",
    &chartInstance->c5_is_active_c5_LaneDetection, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c5_y, 2, c5_d_y);
  sf_mex_assign(&c5_st, c5_y, false);
  return c5_st;
}

static void set_sim_state_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_st)
{
  const mxArray *c5_u;
  c5_u = sf_mex_dup(c5_st);
  c5_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 0)),
                      "bboxes", *chartInstance->c5_bboxes_data,
                      *chartInstance->c5_bboxes_sizes);
  c5_c_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 1)),
                        "scores", *chartInstance->c5_scores_data,
                        *chartInstance->c5_scores_sizes);
  chartInstance->c5_is_active_c5_LaneDetection = c5_e_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c5_u, 2)),
     "is_active_c5_LaneDetection");
  sf_mex_destroy(&c5_u);
  sf_mex_destroy(&c5_st);
}

static void sf_gateway_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
  c5_emxArray_real32_T_20x1 c5_scores;
  c5_emxArray_real_T_20x4 c5_bboxes;
  int32_T c5_i;
  chartInstance->c5_JITTransitionAnimation[0] = 0U;
  _sfTime_ = sf_get_time(chartInstance->S);
  for (c5_i = 0; c5_i < 921600; c5_i++) {
    chartInstance->c5_fv[c5_i] = (*chartInstance->c5_In)[c5_i];
  }

  c5_eML_blk_kernel(chartInstance, chartInstance->c5_fv, c5_bboxes.data,
                    c5_bboxes.size, c5_scores.data, c5_scores.size);
  c5_emxConvertDynamicMatrixFromEmx_(chartInstance, &c5_bboxes,
    *chartInstance->c5_bboxes_data, *chartInstance->c5_bboxes_sizes);
  c5_b_emxConvertDynamicMatrixFromEmx_(chartInstance, &c5_scores,
    *chartInstance->c5_scores_data, *chartInstance->c5_scores_sizes);
  c5_do_animation_call_c5_LaneDetection(chartInstance);
}

static void mdl_start_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static void mdl_terminate_c5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
  cudaError_t c5_errCode;
  cudaFree(*chartInstance->c5_gpu_out_data);
  cudaFree(chartInstance->c5_gpu_i9);
  cudaFree(*chartInstance->c5_gpu_x1_data);
  cudaFree(*chartInstance->c5_gpu_y2_data);
  cudaFree(*chartInstance->c5_b_gpu_scores_data);
  cudaFree(chartInstance->c5_gpu_i17);
  cudaFree(chartInstance->c5_d_gpu_end);
  cudaFree(*chartInstance->c5_b_gpu_out);
  cudaFree(chartInstance->c5_gpu_i32);
  cudaFree(chartInstance->c5_gpu_i39);
  cudaFree(chartInstance->c5_gpu_loop_ub);
  cudaFree(*chartInstance->c5_gpu_anchors);
  cudaFree(*chartInstance->c5_gpu_ipRowIndices);
  cudaFree(chartInstance->c5_gpu_i38);
  cudaFree(*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
  cudaFree(*chartInstance->c5_gpu_thresholdedPrediction_data);
  cudaFree(chartInstance->c5_gpu_scores_data);
  cudaFree(chartInstance->c5_b_gpu_n);
  cudaFree(*chartInstance->c5_gpu_iv);
  cudaFree(chartInstance->c5_gpu_i10);
  cudaFree(chartInstance->c5_gpu_i12);
  cudaFree(chartInstance->c5_c_gpu_end);
  cudaFree(*chartInstance->c5_gpu_aux1);
  cudaFree(*chartInstance->c5_b_gpu_bboxPred_data);
  cudaFree(*chartInstance->c5_gpu_thresholdedPrediction_size);
  cudaFree(chartInstance->c5_gpu_n);
  cudaFree(*chartInstance->c5_gpu_bv);
  cudaFree(*chartInstance->c5_gpu_iv1_size);
  cudaFree(*chartInstance->c5_gpu_ii_data);
  cudaFree(chartInstance->c5_b_gpu_xoffset);
  cudaFree(chartInstance->c5_g_gpu_loop_ub);
  cudaFree(chartInstance->c5_gpu_i35);
  cudaFree(chartInstance->c5_gpu_bboxes_data);
  cudaFree(*chartInstance->c5_gpu_aux2);
  cudaFree(chartInstance->c5_gpu_xoffset);
  cudaFree(chartInstance->c5_gpu_i50);
  cudaFree(chartInstance->c5_gpu_outVal);
  cudaFree(*chartInstance->c5_gpu_out);
  cudaFree(*chartInstance->c5_gpu_colWeightsTotal);
  cudaFree(*chartInstance->c5_gpu_colWeights);
  cudaFree(*chartInstance->c5_gpu_index_data);
  cudaFree(*chartInstance->c5_gpu_ii_size);
  cudaFree(*chartInstance->c5_gpu_scorePred_data);
  cudaFree(*chartInstance->c5_gpu_dv);
  cudaFree(chartInstance->c5_gpu_nrows);
  cudaFree(chartInstance->c5_gpu_nx);
  cudaFree(*chartInstance->c5_gpu_boxOut);
  cudaFree(chartInstance->c5_h_gpu_loop_ub);
  cudaFree(*chartInstance->c5_gpu_rowWeights);
  cudaFree(*chartInstance->c5_gpu_dv1);
  cudaFree(chartInstance->c5_gpu_i14);
  cudaFree(*chartInstance->c5_gpu_bboxesX1Y1X2Y2_size);
  cudaFree(*chartInstance->c5_c_gpu_bboxPred_data);
  cudaFree(chartInstance->c5_gpu_count);
  cudaFree(*chartInstance->c5_gpu_scores_size);
  cudaFree(*chartInstance->c5_gpu_selectedIndex_size);
  cudaFree(*chartInstance->c5_b_gpu_bboxPred_size);
  cudaFree(*chartInstance->c5_gpu_dv2);
  cudaFree(chartInstance->c5_gpu_i51);
  cudaFree(*chartInstance->c5_e_gpu_bboxPred_size);
  cudaFree(chartInstance->c5_gpu_i4);
  cudaFree(chartInstance->c5_b_gpu_count);
  cudaFree(*chartInstance->c5_gpu_area_data);
  cudaFree(*chartInstance->c5_gpu_ipColIndices);
  cudaFree(*chartInstance->c5_gpu_iv1_data);
  cudaFree(chartInstance->c5_gpu_i19);
  cudaFree(*chartInstance->c5_gpu_scorePred_size);
  cudaFree(*chartInstance->c5_gpu_classPred_size);
  cudaFree(*chartInstance->c5_gpu_rowWeightsTotal);
  cudaFree(*chartInstance->c5_gpu_tmpFeatureMap);
  cudaFree(*chartInstance->c5_gpu_bboxPred_data);
  cudaFree(*chartInstance->c5_d_gpu_bboxPred_data);
  cudaFree(chartInstance->c5_d_gpu_loop_ub);
  cudaFree(chartInstance->c5_gpu_i6);
  cudaFree(*chartInstance->c5_gpu_bboxPred_size);
  cudaFree(chartInstance->c5_gpu_i64);
  cudaFree(*chartInstance->c5_gpu_y1_data);
  cudaFree(chartInstance->c5_gpu_i66);
  cudaFree(chartInstance->c5_gpu_end);
  cudaFree(*chartInstance->c5_d_gpu_bboxPred_size);
  cudaFree(chartInstance->c5_gpu_i47);
  cudaFree(chartInstance->c5_gpu_i3);
  cudaFree(*chartInstance->c5_gpu_partialResize);
  cudaFree(*chartInstance->c5_gpu_x1_size);
  cudaFree(chartInstance->c5_c_gpu_n);
  cudaFree(*chartInstance->c5_gpu_idx_data);
  cudaFree(chartInstance->c5_gpu_i);
  cudaFree(*chartInstance->c5_e_gpu_bboxPred_data);
  cudaFree(chartInstance->c5_b_gpu_loop_ub);
  cudaFree(chartInstance->c5_b_gpu_end);
  cudaFree(*chartInstance->c5_c_gpu_bboxPred_size);
  cudaFree(*chartInstance->c5_gpu_b_data);
  cudaFree(chartInstance->c5_e_gpu_loop_ub);
  cudaFree(*chartInstance->c5_gpu_x2_data);
  cudaFree(*chartInstance->c5_gpu_classPred_data);
  cudaFree(chartInstance->c5_gpu_i7);
  cudaFree(*chartInstance->c5_b_gpu_anchors);
  cudaFree(chartInstance->c5_f_gpu_loop_ub);
  cudaFree(*chartInstance->c5_b_gpu_outVal);
  cudaFree(chartInstance->c5_gpu_i61);
  cudaFree(*chartInstance->c5_gpu_b_size);
  cudaFree(chartInstance->c5_gpu_i45);
  cudaFree(*chartInstance->c5_gpu_In);
  cudaFree(chartInstance->c5_c_gpu_loop_ub);
  cudaFree(chartInstance->c5_gpu_y);
  cudaFree(*chartInstance->c5_gpu_selectedIndex_data);
  c5_errCode = cudaGetLastError();
  if (c5_errCode != cudaSuccess) {
    emlrtThinCUDAError(c5_errCode, cudaGetErrorName(c5_errCode),
                       cudaGetErrorString(c5_errCode), "SimGPUErrorChecks",
                       chartInstance->c5_fEmlrtCtx);
  }
}

static void mdl_setup_runtime_resources_c5_LaneDetection
  (SFc5_LaneDetectionInstanceStruct *chartInstance)
{
  setLegacyDebuggerFlag(chartInstance->S, false);
  setDebuggerFlag(chartInstance->S, false);
  sim_mode_is_external(chartInstance->S);
}

static void mdl_cleanup_runtime_resources_c5_LaneDetection
  (SFc5_LaneDetectionInstanceStruct *chartInstance)
{
}

static void initSimStructsc5_LaneDetection(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
}

const mxArray *sf_c5_LaneDetection_get_eml_resolved_functions_info()
{
  const mxArray *c5_nameCaptureInfo = NULL;
  const char_T *c5_data[45] = {
    "789ced9dc973ebc87dc739ae7132763c338a27138f5359de547289277922a985922f3109921229eea4c4e555ea09049b24441000b170bb84b9b92a871c928353"
    "d9173b7179c9a4ca4965ad5cf327e43fc835e5532eae0ab8b444e2110f1c35d90f6afe50358307fda0fefe7a517fd0bbefad44fa2d9fcff79ef5df1ffd8ecff7",
    "5f5ffb9c6f7abdeb9b5f078bfbe77cab97ddfed6e25eb53de3ebf3beb7577e6f6aff434beff717cf82221b6868cc1f64be8bee7fb3a1744599978dd248453e0d"
    "e98ad4478d99a5294aa824765171f921337deac6974cf70f53d3f4df5c1b099da2d9f5696dfdc14369f961961ed3ebd621be6fbba487fdb2a787fd3dac3779a4",
    "1e0effab2e7ad8de476d5190d0cb06329060888afc72a448cacb7ed0e6cf2da13f3fe5e8cfdca28b724b420f7adf22d43b72d45bb5bf88fdf6615be9a2c37657"
    "900ec3d169f9d314e9d0315d9e7767bfe7962eef6fe8a7fdfef0fe3bb3fb8ffef3ed9734f5f0b52f7a4387f0362d673fefa07760b39ba55c878ba3782e71153b",
    "e713b562af9c6fc51efcc8b9e8b8f9e17378a615fe7f3bfcfea6e9d87308ffc0667f91a8587fb1a6ae1d4a8ac04b87e97029158e1c1682fea0bf7e68288a5457"
    "86877a9bd750e3b021094a03692febbc8e561f3e99fdfbf013d1a28d265be1480adf8822a4a610afc9565d9441c640d13acf555b3c49cbcb072ef1c4f6997fcf",
    "abd954b61f5cf8b2e2c72da11f6fdb9e1ffc985b042b0597f5fe9750eff71cf556ed2f12dc72851c5504b38b6443c7195d345555d18c1c2f74f816d237c9787d"
    "fe2b2afe95f93b2f1bd24b83d75ac80a7b511a56527b91f3b4eaa55fffc9c750cfef508ff4eff64307bd039bfda6d4baeb9d1f5f5db5e315a1de3ff5378ee249",
    "1f3bf5fcbed703df248cffc72ef1c7f62e6f487c9db3fc692139637d949a325f97504e5354a41922c26d14daed853997d690d2e6cf3b6f91f9f31d177fb0fd4d"
    "96936fac498643878f085af5ded73ffd317064877ab4da0b9d41ab5732b96c2d1416f3859b40a5c9e7ef2e8023ac7004b72b1e1bff776dcff6f863bbe5cfc281",
    "6cfdce07bc005ee0f7811734f48017db091f784116ff2fb9c41fdb2d7f4a33af52627d597f42a80fbc005e3c460f5ffba207bcd84ef8fbce0bd272f4b32ef1c7"
    "f67b5e4479839f0dcdfb56fc9810fa01dc006e3c460f5ffba207dcd84ef8fbce0dd276c6175de28fed963f89ec743463319001bc005eccdf075ed0d0035e6c27",
    "fc7de7056939fa8a4bfcb15d98ced58d2b5a11f54c240b08d7d6c00de0c6fc7de0060d3de0c676c2df776e90b6339cd2fbc0767f686758f0286432c00be0c5fc"
    "7de0050d3de0c576c2075e90c5ff5ddbb33dfed83eb06aa284ac9a46586be93ee897025ee0f7811734f48017db097fdf79714b18ff776ccf0ff19f5ba633a4f8",
    "11d2800fc087f9fbc0071a7ac087ed84bfef7c206d4f7c8671ee9ce56cc6ecfa56f42784fac00be0c563f4f0b52f7ac08bed840fbc208bffcfb8c41fdb071aaf"
    "a6f9d96cda15fd09a13ef00278f1183d7ced8b1ef0623be1ef3b2f2684f1ff0597f8637b9f97c4066fe0f9507145e3245ed7c5e6c8b7557f801fc08fc7e8e16b",
    "5ff4801fdb097fdff941daded874bcdbf267068c0cdf4530de0dbc78781f7841430f78b19df0f79d1713c2f86fdade58d99730a50c9056528a86660a8b2ddb81"
    "1fc08ff9fbc00f1a7ac08fed840ffc208bffa6f5b5aa21d5726f36c5b6288ed174590687fabcb4b0033f801ff3f7811f34f4801fdb097fdff941da5ff59eedd9",
    "1e7f6cd7eff7b3cdcc0e87025e002fe6ef032f68e8012fb613febef3e29630fe9f778cffdc6271c2547dc007e0037e1ff840430ff8b09df0f79d0fa4ed892fb8"
    "c41fdb055e928a0b562ceb4f08f58117c08bc7e8e16b5ff48017db091f784116ff4dd7672c78311dba58d19f10ea032f80178fd1c3d7bee8012fb6133ef0822c",
    "fe9b8e574c795140ba450c833760bc027871ff3ef082861ef0623be1032fc8e2ffbeedd91e7f6c9ff2e25a9daee89b03037801bc98bf0fbca0a107bcd84ef8c0"
    "0bb2f86f7a5eaba1f1b2ae2a3abae1b565fd09a13ef00278f1183d7ced8b1ef0623be1032fc8e2ef94de07b6bb8ab4ae69ccd75f4c3799025e002fe6ef032f68",
    "e8012fb613febef382d6f9de0b5e644d63018cc5cf811bc08df9fbc00d1a7ac08ded840fdc208bff072ef1c776a79a1ab801dc98bf0fdca0a107dcd84ef8fbce"
    "0dd2fea94dcfcbd096e64a2deb4f08f5b7cd0b1f212ffeccc51f6c7f91c87a0c180f394493139ffce463e0c40ef56871e2725cad760ae55afda6148cf377c29d",
    "5e2fc422c0095638415a8e3e74893fb6ab966ba26084e5c6f26c29eff2e2809017df75f107dbbdc78bb539f51c7f4fd0aaefbe06fcd8a91e2d7e1c9de7838db1"
    "de2e1cc56b3139356a9ae7a5cc25f083157edc12c6dfeddc5661713a86573941daaef813177fb0dd7b9cc039332f10b4eab1df002eec548f161762caf0341fbf",
    "29f851e8e22e533fbd0c8c6fa2d0ae60860ba4fd4f9baeefe60543ecf386a8c8fa8afe8450df6b9cf873177fb0dd7b9c58ca21ab8c40ff131b7ab438d1bf8ed5"
    "8ba181c225536643484823a3daedc03805339cb8258cff4f3bc67f6e59f46630cb853f76f107dbbdc78545ce6c581e800b4f438f16177ac50b29378c718d4aa1",
    "a5e5913fad0c12fd387081152e4c08e3bfe9f948d3f5dbb9794d343dd9c2d40da59be247489b37273ccb0dd2f1891fbaf883eddee386738ed11cdffefebffd2e"
    "3fbdef4bbd4e5b8f1647b4cb74de8834ce8f934a6ad40b9fb4fa89660ada17cc7084d6ba8ba55e0eab562a64328b9f7b951fa4f367bfede20fb67b8f1fafe4d4",
    "acc4c0f8051b7ab4b8c12505d49244291f112ad14e2e7add4cb506307e01dc58dc1fc70d0eb8b1b87b9d1b1c656e40bfd56ef5687123733cd2f345a3726795ea"
    "52b77f3948079563e00633dc5009e3ffaeedd91e7f6c57ef7b40e66d0d5679f1972efe60bbf778b19243d4e7cf022f76ab476bde7cbb90354b576222276bc745",
    "4d0e1f19c6d9f2bc79e0c5faf09f0a2f6895a369af79784d1f9557b9b1bfeb2fd6e614f083313d5aed8d51fa5810e59b8aa14891a37eb6df8a0e2a9128f003f8"
    "31bf36eda75a1d7585f10deff653bd9253d04fc5901eb57e2abead16c6e245521c72c3715c572f8cb4c2013758e1c63709e3ff6b2ef1c77655d18d9ca60848d7",
    "e77b12969402324c4de67803b5144db4aaab65bf26847e6d9b231f1172e49f5dfcc176ef7164939c7baed2aa0761dfa9ddead1e24af6f22693eef2d791f35434"
    "735757af82fd4b25065c61852ba4e31fefd99eedf1c7f6d55e12eff283b41df2572efe60bbf7f8b19a43b8030be659b1a147abdffab42c66c352f6e8bad354c6",
    "5553ad56735ac207bc005eccafcf720ef8a25764457f42a8efb5f5804f779df8520ec13a7166f468b52bae73858a7c16cf9606d9da5d463f0bc40d03d6710027"
    "707cbfe0127f6cd791119685b6a22d9e59e5c49fbaf883eddee3c47d0ec1ba0d86f4a8addb101267a38674dd4aa27aa77769ea6669948675e3c089c57dd373bf",
    "ad5a68e140511c23ddb39c20ed7ffa6b177fb0dd939c58ce21cafb8c002f76ab478b1721a9a5fb8ff4b891499785ea6994d7e5a80af3a798e1c58430fecf5ce2"
    "8fed2aafe9f3535c75ce72aa85e4a5399e854cc6abfc201dfffe918b3fd8ee3d7eb8e518ec7bcb921e2d9e94f263a39ee6c37d2e76d28d7486423e9d45d04f05",
    "3c59dc9fb9c41fdb5f5b3b71c09327c6138e364f60dc63b77ab478e26f9adac9406db72e338948229fec5e9d6b12ccd3059e2ceebfe8127f6c7fb5765a8cc52e"
    "b624f12a4f48d7097eeae20fb63f059e3ce418bd79b9c091ddead1e2c869ba943fabc653c391903f6974f93b7e702dc1be24c091c59d982305e0c8f47a421c29",
    "d0e408f46fed568f1647aec2d245711039354efccd44554b4b6533da80f608331c212d475f76893fb6ebc898d549d3b1dbe505e75ee507e938fb775cfcc176ef"
    "f1e3d59ca2b90f3b9c13bb5b3d5adc08f24629143e1e9f66b890d2f1e785512fc343fb83196e505b6f3e5fb1bc5c23addbf6cab31cd9e3f5e61be41cb44758d1",
    "a3c5153369648737839479913e2ea042dbef4f4a7d98bfc50c572684f17fe6127f6c5f533bad6cb2c42a4f9ef078bb4b8ec1fc2d96f468ad478fd49222675c73"
    "a73c1fe333b5b13f9440a60f78c20a4f54c2f87fc925fed83eeb755faab2bdca0f1f213ffec2c51f6cf7203f967208c6d359d1a3d5ee509bfdf2305c3aefc67a",
    "c7b2a09fc702b15e18d61932c3095adf1b963fcb9513a7c84db1e5f32e2ff677dff6b53905fbb633a6476d3e967a1ecda5afda665734438974f6eed8c8342f81"
    "1facf0634218ff6dd7dbb784febc6d7b7ef0676e11ac445ad6a3375eee314cacf919cdf172d84777b77ab4f8d019b47a2593cbd64261315fb809549a7cfe8ea1",
    "7584b70ebfef968ef6cb291df74def5b8fd4c3e19fb9e861fb8bebe5ea361c1514d9d014e970a4484a3f5840ba55e79df86f505b1424141bf25d5542cfbbbc41"
    "ebeff57fbef8617d7aa7a5f78dbf7916a4a9872fd6ebbfe1d9b8df6bd423e96645abd7c4ec695fd0820ced23fe8cf0fbe8df1dc23fb0d95f24624fe0fbe863b1",
    "805aa26e202d8a542437902c88485f4eaf89437a3cb5ef69fbb569fd0edfd3f03dcda21e7c4f6f277ce0c967e30969b9fbc825bdb0bd212d22f15c301bb23cff"
    "7f5815df1447feef917a38fc3f70d1c3760a1c69a9e6dcba5c52eed3db2a34d3a43ec4094e797fc0df827efa9dead1e2060a0522914c5f38efa7f277811ca736",
    "dba10643f34b7d84dc78bae711f00f93cf97ce23a0c585351e5d1ba2a46fabbdf3cb2e7e60bb637b67e6cd9b583fedad62324b86751f193303bd7943bf093cd9"
    "a91eb5715f2159ac08e5b01f5ddcf8c77a30d73e2ef10ccd1b029eacf2447588efa6e9f1aeedd99e1ed86eb9772d8b3d1365f82ef26dafdf6cdb1cd9db75d473",
    "90ac6413cd7e2c589fb05b3d5afc08f4c6d1d1f0ba5a0a36514048449af5a3f33243eba8811fabfca0b7ae1aa996b7374830142d6b1ac5d9c8edea1adde97b5e"
    "e50a69ffe77fb8f883ed9e2b3773ae6c927d54cf5382f6ca6ef568cd4faf69422d64d650b39e0f1b52d5d432a6b4fc7d09bc591ffebef2e6575dd203db171556",
    "a26b79fe1adc30cb9bcdc7db3c566e5678f3baeca3bbfe1a78b35b3d5abc0966e460a72ef55ae75ca95dca8692d2455ef1016f58e50d69b9fa3997f4c0f64585"
    "55c8643824495695a59ab383c2bdca17d2f1961fb8f883ed9e2b272b7c59c92e380f96213d5afd654583ef0e846be13a864ecf637a9c4b17840a8cdf33cb9389",
    "437c374d8f5f72490f6cc715943248f377b3fd86e67d2fb3ed887cdee50ae97e51ffe4e20fb67baebcac72656db6d11c8f81f6ca6ef568f1a5e21f5d368367dc"
    "117f9a48e6839dab5eaa5086f17ce08b437a6c7ccec62b15d5acaf05e38559befca38b3fd8eeb9f2e2c497876c83f11786f468f1e5bc9d93f9abb8297181a3dc",
    "a8de4a9f24cb3243eb20812fdbe5cb3397f4c0f64545c52992d995d77c0b7b952fa4e32effe2e20fb67baebcacf0c531dba8ef47059cd9ad1e2dce08d96665ac"
    "1c0bb1a3b34eaf9b0f06caf52304ed18e08c437afc8a4b7a60fbda0a6ba929e355ceeced391dafe1cc725306f8c2861ead71fd4ba32624a2e7f291a146c7c178",
    "bfdc4f2597fbc9812febc37faa7ca1b60ff3bca24a8bb258e70da18d4f80f22a5748c7f5ffdec51f6cf75c3959e18a2dbb96cacdad43fc81274f438fd6dfbda1"
    "26e2c190209e57b57ed18c041b63652430b47f2ef064bbf3923f76490f6c5f5450f319474e870bfabccb17d2feb17f75f107db3d576e56f8e29c7db8830c78c3",
    "861e2ddea4aeeeccc0492e2955a4bb7ce926702ef78d1af08659de4c1ce2bba37964f715d6ca11763eef7206e69139671bcc2363478f56ff58777452e2aad953"
    "7fa63ece348db8303851e23ee00bab7c212d571fb8a407b65bee71a66e28dd143f425635a5a8de5e5749da3ff67d177fb0dd73e5e47e7f187b76c13a4a96f468",
    "f124c99f76ae1017085dc68d642577aca58eaf601d25bb3c511de2bbed7dc68436123a89665c911aa8e1f32e47f67c9fb1956c82f6083b7ab4fabbaa72a8d6aa"
    "7745a326ea83546778c21b013fac9b64961f14cfbb9d779a14c5f174c390d977aecfbb1c216d8ffcd0c51f6cf75c39b96f8fbc9a5dc01376f468f1241b1a5d84",
    "11874e1aa5d8553a789ccf1c9931183f019e38a4c7975dd203db2df712d9593fc9124c7cecf2e47b2efe60bbe7cac93d4f6cd945f95c16e0c96ef568f1a49d18"
    "66c3f29dbf171b6793c746489706a32eac57019e38a4c7575cd203db2df78a96efa891c8ceaaa784dc1005a433cb934f5dfcc176cf95937b9eaccb2e589fc28a",
    "1e2d9ee85701b1a55f268781e35c3a77640cf3a3761ff6d5679627b4c64b2cf770c584a665d9ab1cd9f3f192956c82fe2d76f468f56bf74ab94ab0c0252fdb89"
    "58ab9b1107b29aaffa801fc08ff5e9e194fe07b6bbe5de74b15c64ba582e213715aff283b41df25d177fb0dd73e5e39e1f2bd904fbb030a647edbce164e7b466",
    "9ae1403f5817f5713adf38896bd0afc52c47260ef1ddf63a93694709ea994816500ac92da3bdd85a1df62b9e5f9e2b2f0ffd5b8ed906ed1476f468f1e5eae6ae"
    "5c8a7623dd71161931ed5ae3ce9b230ef8c22a5f48db29efdb9eede981edf71385162b17bdca13d27eaebf73f107db3d573e6cf3b9f04a45e0071b7ab4fab9c6",
    "bc2195ca5af8fcc2ecf574433c2b0eb8880ff8c12a3fa8cf0b8e23de3035349d6f6a7d96789623302ff8d5ec82f6083b7ab4da235c55f76b3791eb536e58afb6"
    "afa5c245404dc3bc60667942eb3cfb7515d49aed553ccb976784e5e6899f67bf49f6c1792a0ce9d1e24d211af7d7fa67f971f74eb918a14e2a135663709e0ab3",
    "bca1386f7865a1dca2a6f22c5f60def0baec82fe3056f468f55b148ab5e351a434ac72ddb034ca075aa1467b795f0ce0c9faf0f795279b9e573fdfc0437ea8a1"
    "3cde5ed9f3f3ead765179c57cf901e2d9ed4ce2ffbd59b7e2b7e2d4ae57eed4ad6d289820f78c22a4f5487f86e9a1eefd99eede981edab15947739423a3effb7",
    "2efe60bbe7cac73a8ec0b928cce8d1eadf1a5c05eeae6bd152881fa79be56c3913383178d8b78b597ed0fa2ee9f392d8e00d8467a37a7ddef09e8fcfafcd2e18"
    "9f67478f164f8a1dfe663c54a2c37279d83acaa463d163b3043c619627a4ed11a7f43fb0dd7105355b751dd65aac72e489af6bb46713f5758dd0afb55b3d5a1c",
    "b9098c5b6d34ec1e952fa201d98c5c85f44ae50238c22a4748cbd5472ee981edb30e93b0dc285b4ec515ed92979ab36f5daff284743de33fb8f883ed9e2b274b"
    "fd5b6bb20bd6cb33a647ab5fa23e5646b9b62e551b9715f5aa963fca14f230fece2e57260ef17d53f5f9ed23fdb15f4efee08bfebc2d6f1587393e9c0c306f8b",
    "153d5aed91532159ac08e5b01f5ddcf8c77a30d73e2ef1b0cf0a70c3213dbeea921ed8eec88d85fd4d7183defe8e1e2b0e6b7e46731ce4eb9ffe1878b1433d5a"
    "bce80c5abd92c9656ba1b0982fdc042a4d3e7fc750ff1569fdf06d87f00f6c764ff322ae689c6ddee6c421dec08dd7eb01375e9f2ec08d37ab07dcd84ef8c08d",
    "dd7003faa95eaf07fd54ebd307faa9e8e8413f1559f8ff0fa87abd35", "" };

  c5_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c5_data[0], 142048U, &c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_eML_blk_kernel(SFc5_LaneDetectionInstanceStruct *chartInstance,
  real32_T c5_b_In[921600], real_T c5_b_bboxes_data[], int32_T c5_bboxes_size[2],
  real32_T c5_b_scores_data[], int32_T c5_scores_size[2])
{
  static real_T c5_dv1[4] = { 3.6875, 1.375, 1.8125, 6.8125 };

  static int8_T c5_dv[8] = { 59, 22, 29, 109, 43, 18, 23, 84 };

  c5_yolov2ResNet50VehicleExample0_LaneDetection0 *c5_iobj_0;
  c5_yolov2ResNet50VehicleExample0_LaneDetection0 *c5_this_Network;
  dim3 c5_ab_block;
  dim3 c5_ab_grid;
  dim3 c5_b_block;
  dim3 c5_b_grid;
  dim3 c5_bb_block;
  dim3 c5_bb_grid;
  dim3 c5_block;
  dim3 c5_c_block;
  dim3 c5_c_grid;
  dim3 c5_cb_block;
  dim3 c5_cb_grid;
  dim3 c5_d_block;
  dim3 c5_d_grid;
  dim3 c5_db_block;
  dim3 c5_db_grid;
  dim3 c5_e_block;
  dim3 c5_e_grid;
  dim3 c5_eb_block;
  dim3 c5_eb_grid;
  dim3 c5_f_block;
  dim3 c5_f_grid;
  dim3 c5_fb_block;
  dim3 c5_fb_grid;
  dim3 c5_g_block;
  dim3 c5_g_grid;
  dim3 c5_gb_block;
  dim3 c5_gb_grid;
  dim3 c5_grid;
  dim3 c5_h_block;
  dim3 c5_h_grid;
  dim3 c5_hb_block;
  dim3 c5_hb_grid;
  dim3 c5_i_block;
  dim3 c5_i_grid;
  dim3 c5_ib_block;
  dim3 c5_ib_grid;
  dim3 c5_j_block;
  dim3 c5_j_grid;
  dim3 c5_jb_block;
  dim3 c5_jb_grid;
  dim3 c5_k_block;
  dim3 c5_k_grid;
  dim3 c5_kb_block;
  dim3 c5_kb_grid;
  dim3 c5_l_block;
  dim3 c5_l_grid;
  dim3 c5_lb_block;
  dim3 c5_lb_grid;
  dim3 c5_m_block;
  dim3 c5_m_grid;
  dim3 c5_mb_block;
  dim3 c5_mb_grid;
  dim3 c5_n_block;
  dim3 c5_n_grid;
  dim3 c5_nb_block;
  dim3 c5_nb_grid;
  dim3 c5_o_block;
  dim3 c5_o_grid;
  dim3 c5_ob_block;
  dim3 c5_ob_grid;
  dim3 c5_p_block;
  dim3 c5_p_grid;
  dim3 c5_pb_block;
  dim3 c5_pb_grid;
  dim3 c5_q_block;
  dim3 c5_q_grid;
  dim3 c5_qb_block;
  dim3 c5_qb_grid;
  dim3 c5_r_block;
  dim3 c5_r_grid;
  dim3 c5_rb_block;
  dim3 c5_rb_grid;
  dim3 c5_s_block;
  dim3 c5_s_grid;
  dim3 c5_sb_block;
  dim3 c5_sb_grid;
  dim3 c5_t_block;
  dim3 c5_t_grid;
  dim3 c5_tb_block;
  dim3 c5_tb_grid;
  dim3 c5_u_block;
  dim3 c5_u_grid;
  dim3 c5_ub_block;
  dim3 c5_ub_grid;
  dim3 c5_v_block;
  dim3 c5_v_grid;
  dim3 c5_vb_block;
  dim3 c5_vb_grid;
  dim3 c5_w_block;
  dim3 c5_w_grid;
  dim3 c5_wb_block;
  dim3 c5_wb_grid;
  dim3 c5_x_block;
  dim3 c5_x_grid;
  dim3 c5_y_block;
  dim3 c5_y_grid;
  real_T c5_b_bboxPred_data[3136];
  real_T c5_bboxPred_data[3136];
  real_T c5_bboxesX1Y1X2Y2_data[3136];
  real_T c5_area_data[784];
  real_T c5_x2_data[784];
  real_T c5_y1_data[784];
  real_T c5_y2_data[784];
  real_T c5_areaOfIntersect;
  real_T c5_count;
  real_T c5_height;
  real_T c5_width;
  int32_T c5_idx_data[784];
  int32_T c5_b_size[2];
  int32_T c5_bboxPred_size[2];
  int32_T c5_bboxesX1Y1X2Y2_size[2];
  int32_T c5_d_bboxPred_size[2];
  int32_T c5_e_bboxPred_size[2];
  int32_T c5_inDims[2];
  int32_T c5_iv[2];
  int32_T c5_iv1[2];
  int32_T c5_thresholdedPrediction_size[2];
  int32_T c5_b_bboxPred_size[1];
  int32_T c5_b_scores_size[1];
  int32_T c5_c_bboxPred_size[1];
  int32_T c5_classPred_size[1];
  int32_T c5_ii_size[1];
  int32_T c5_index_size[1];
  int32_T c5_iv1_size[1];
  int32_T c5_scorePred_size[1];
  int32_T c5_selectedIndex_size[1];
  int32_T c5_x1_size[1];
  int32_T c5_x2_size[1];
  int32_T c5_y1_size[1];
  int32_T c5_y2_size[1];
  int32_T c5_b_end;
  int32_T c5_b_i;
  int32_T c5_b_i14;
  int32_T c5_b_i7;
  int32_T c5_b_j;
  int32_T c5_b_k;
  int32_T c5_b_k0;
  int32_T c5_b_n;
  int32_T c5_b_nx;
  int32_T c5_b_nxin;
  int32_T c5_b_partialTrueCount;
  int32_T c5_b_trueCount;
  int32_T c5_c_i;
  int32_T c5_c_j;
  int32_T c5_c_k;
  int32_T c5_c_n;
  int32_T c5_c_partialTrueCount;
  int32_T c5_c_trueCount;
  int32_T c5_currentBox;
  int32_T c5_d_i;
  int32_T c5_d_k;
  int32_T c5_e_i;
  int32_T c5_e_k;
  int32_T c5_end;
  int32_T c5_f_i;
  int32_T c5_f_k;
  int32_T c5_g_i;
  int32_T c5_g_k;
  int32_T c5_h_i;
  int32_T c5_h_k;
  int32_T c5_i;
  int32_T c5_i1;
  int32_T c5_i10;
  int32_T c5_i11;
  int32_T c5_i12;
  int32_T c5_i13;
  int32_T c5_i14;
  int32_T c5_i17;
  int32_T c5_i19;
  int32_T c5_i25;
  int32_T c5_i26;
  int32_T c5_i27;
  int32_T c5_i3;
  int32_T c5_i32;
  int32_T c5_i35;
  int32_T c5_i39;
  int32_T c5_i4;
  int32_T c5_i47;
  int32_T c5_i5;
  int32_T c5_i51;
  int32_T c5_i6;
  int32_T c5_i61;
  int32_T c5_i64;
  int32_T c5_i66;
  int32_T c5_i7;
  int32_T c5_i8;
  int32_T c5_i9;
  int32_T c5_i_i;
  int32_T c5_idx;
  int32_T c5_ii;
  int32_T c5_j;
  int32_T c5_j_i;
  int32_T c5_k;
  int32_T c5_k0;
  int32_T c5_n;
  int32_T c5_nrows;
  int32_T c5_nrowx;
  int32_T c5_nx;
  int32_T c5_nxin;
  int32_T c5_nxout;
  int32_T c5_partialTrueCount;
  int32_T c5_sortDim;
  int32_T c5_trueCount;
  real32_T c5_tmpFeatureMap[4704];
  real32_T c5_c_scores_data[784];
  real32_T c5_classPred_data[784];
  real32_T c5_scorePred_data[784];
  real32_T c5_outVal[2];
  real32_T c5_y;
  uint32_T c5_dv2[2];
  int16_T c5_ii_data[784];
  int16_T c5_iv1_data[784];
  boolean_T c5_b_data[784];
  boolean_T c5_bv[784];
  boolean_T c5_index_data[784];
  boolean_T c5_selectedIndex_data[784];
  boolean_T c5_ab_validLaunchParams;
  boolean_T c5_area_data_dirtyOnGpu;
  boolean_T c5_b_bboxPred_data_dirtyOnGpu;
  boolean_T c5_b_bboxPred_size_dirtyOnCpu;
  boolean_T c5_b_data_dirtyOnCpu;
  boolean_T c5_b_data_dirtyOnGpu;
  boolean_T c5_b_n_dirtyOnGpu;
  boolean_T c5_b_validLaunchParams;
  boolean_T c5_bb_validLaunchParams;
  boolean_T c5_bboxPred_data_dirtyOnCpu;
  boolean_T c5_bboxPred_data_dirtyOnGpu;
  boolean_T c5_bboxPred_size_dirtyOnCpu;
  boolean_T c5_bboxesX1Y1X2Y2_data_dirtyOnGpu;
  boolean_T c5_bboxesX1Y1X2Y2_size_dirtyOnCpu;
  boolean_T c5_bboxes_data_dirtyOnGpu;
  boolean_T c5_bv_dirtyOnGpu;
  boolean_T c5_c_bboxPred_size_dirtyOnCpu;
  boolean_T c5_c_validLaunchParams;
  boolean_T c5_cb_validLaunchParams;
  boolean_T c5_classPred_data_dirtyOnCpu;
  boolean_T c5_classPred_data_dirtyOnGpu;
  boolean_T c5_d_validLaunchParams;
  boolean_T c5_db_validLaunchParams;
  boolean_T c5_e_validLaunchParams;
  boolean_T c5_eb_validLaunchParams;
  boolean_T c5_exitg1;
  boolean_T c5_f_validLaunchParams;
  boolean_T c5_fb_validLaunchParams;
  boolean_T c5_g_validLaunchParams;
  boolean_T c5_gb_validLaunchParams;
  boolean_T c5_guard1 = false;
  boolean_T c5_h_validLaunchParams;
  boolean_T c5_hb_validLaunchParams;
  boolean_T c5_i_validLaunchParams;
  boolean_T c5_ib_validLaunchParams;
  boolean_T c5_idx_data_dirtyOnGpu;
  boolean_T c5_ii_data_dirtyOnCpu;
  boolean_T c5_index_data_dirtyOnGpu;
  boolean_T c5_iv1_data_dirtyOnCpu;
  boolean_T c5_j_validLaunchParams;
  boolean_T c5_jb_validLaunchParams;
  boolean_T c5_k_validLaunchParams;
  boolean_T c5_kb_validLaunchParams;
  boolean_T c5_l_validLaunchParams;
  boolean_T c5_lb_validLaunchParams;
  boolean_T c5_m_validLaunchParams;
  boolean_T c5_mb_validLaunchParams;
  boolean_T c5_n_dirtyOnCpu;
  boolean_T c5_n_dirtyOnGpu;
  boolean_T c5_n_validLaunchParams;
  boolean_T c5_nb_validLaunchParams;
  boolean_T c5_nrows_dirtyOnGpu;
  boolean_T c5_o_validLaunchParams;
  boolean_T c5_ob_validLaunchParams;
  boolean_T c5_p_validLaunchParams;
  boolean_T c5_pb_validLaunchParams;
  boolean_T c5_q_validLaunchParams;
  boolean_T c5_qb_validLaunchParams;
  boolean_T c5_r_validLaunchParams;
  boolean_T c5_rb_validLaunchParams;
  boolean_T c5_s_validLaunchParams;
  boolean_T c5_sb_validLaunchParams;
  boolean_T c5_scorePred_data_dirtyOnCpu;
  boolean_T c5_scorePred_data_dirtyOnGpu;
  boolean_T c5_scores_data_dirtyOnCpu;
  boolean_T c5_scores_data_dirtyOnGpu;
  boolean_T c5_selectedIndex_data_dirtyOnCpu;
  boolean_T c5_selectedIndex_data_dirtyOnGpu;
  boolean_T c5_t_validLaunchParams;
  boolean_T c5_tb_validLaunchParams;
  boolean_T c5_thresholdedPrediction_size_dirtyOnCpu;
  boolean_T c5_u_validLaunchParams;
  boolean_T c5_ub_validLaunchParams;
  boolean_T c5_v_validLaunchParams;
  boolean_T c5_validLaunchParams;
  boolean_T c5_vb_validLaunchParams;
  boolean_T c5_w_validLaunchParams;
  boolean_T c5_wb_validLaunchParams;
  boolean_T c5_x1_size_dirtyOnCpu;
  boolean_T c5_x2_data_dirtyOnGpu;
  boolean_T c5_x_validLaunchParams;
  boolean_T c5_y1_data_dirtyOnGpu;
  boolean_T c5_y2_data_dirtyOnGpu;
  boolean_T c5_y_validLaunchParams;
  c5_scores_data_dirtyOnCpu = false;
  c5_iv1_data_dirtyOnCpu = false;
  c5_selectedIndex_data_dirtyOnCpu = false;
  c5_b_data_dirtyOnCpu = false;
  c5_scorePred_data_dirtyOnCpu = false;
  c5_classPred_data_dirtyOnCpu = false;
  c5_bboxPred_data_dirtyOnCpu = false;
  c5_ii_data_dirtyOnCpu = false;
  c5_scores_data_dirtyOnGpu = false;
  c5_bboxes_data_dirtyOnGpu = false;
  c5_index_data_dirtyOnGpu = false;
  c5_area_data_dirtyOnGpu = false;
  c5_selectedIndex_data_dirtyOnGpu = false;
  c5_b_data_dirtyOnGpu = false;
  c5_idx_data_dirtyOnGpu = false;
  c5_scorePred_data_dirtyOnGpu = false;
  c5_classPred_data_dirtyOnGpu = false;
  c5_bboxPred_data_dirtyOnGpu = false;
  c5_b_bboxPred_data_dirtyOnGpu = false;
  c5_y2_data_dirtyOnGpu = false;
  c5_x2_data_dirtyOnGpu = false;
  c5_y1_data_dirtyOnGpu = false;
  c5_bboxesX1Y1X2Y2_data_dirtyOnGpu = false;
  if (!chartInstance->c5_yolodetector_not_empty) {
    c5_iobj_0 = &chartInstance->c5_gobj_0;
    c5_DeepLearningNetwork_setup(chartInstance, &chartInstance->c5_gobj_0);
    chartInstance->c5_yolodetector.Network = c5_iobj_0;
    chartInstance->c5_yolodetector_not_empty = true;
  }

  c5_this_Network = chartInstance->c5_yolodetector.Network;
  c5_eML_blk_kernel_kernel1<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_aux1);
  c5_eML_blk_kernel_kernel2<<<dim3(3U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_aux2);
  c5_eML_blk_kernel_kernel3<<<dim3(4U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_aux1, *chartInstance->c5_gpu_rowWeights,
     *chartInstance->c5_gpu_ipRowIndices);
  c5_eML_blk_kernel_kernel4<<<dim3(6U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_aux2, *chartInstance->c5_gpu_colWeights,
     *chartInstance->c5_gpu_ipColIndices);
  c5_eML_blk_kernel_kernel5<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_rowWeights, *chartInstance->c5_gpu_rowWeightsTotal);
  for (c5_k = 0; c5_k < 8; c5_k++) {
    c5_eML_blk_kernel_kernel6<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*chartInstance->c5_gpu_rowWeights, (c5_k + 1) * 224,
       *chartInstance->c5_gpu_rowWeightsTotal);
  }

  c5_eML_blk_kernel_kernel7<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_colWeights, *chartInstance->c5_gpu_colWeightsTotal);
  for (c5_b_k = 0; c5_b_k < 11; c5_b_k++) {
    c5_eML_blk_kernel_kernel8<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*chartInstance->c5_gpu_colWeights, (c5_b_k + 1) * 224,
       *chartInstance->c5_gpu_colWeightsTotal);
  }

  cudaMemcpy(chartInstance->c5_gpu_In, &c5_b_In[0], 3686400UL,
             cudaMemcpyHostToDevice);
  c5_eML_blk_kernel_kernel9<<<dim3(630U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_colWeightsTotal, *chartInstance->c5_gpu_colWeights, *
     chartInstance->c5_gpu_In, *chartInstance->c5_gpu_ipColIndices,
     *chartInstance->c5_gpu_partialResize);
  c5_eML_blk_kernel_kernel10<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_rowWeightsTotal, *chartInstance->c5_gpu_rowWeights, *
     chartInstance->c5_gpu_partialResize, *chartInstance->c5_gpu_ipRowIndices,
     *chartInstance->c5_gpu_out);
  c5_eML_blk_kernel_kernel11<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_out, *chartInstance->c5_b_gpu_outVal);
  c5_coder_reduce0<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_out, *chartInstance->c5_b_gpu_outVal);
  cudaMemcpy(&c5_outVal[0], chartInstance->c5_b_gpu_outVal, 8UL,
             cudaMemcpyDeviceToHost);
  c5_y = c5_outVal[1] - c5_outVal[0];
  c5_eML_blk_kernel_kernel12<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>(c5_y,
    c5_outVal[0], *chartInstance->c5_gpu_out, *chartInstance->c5_b_gpu_out);
  cudaMemcpy(&chartInstance->c5_out[0], chartInstance->c5_b_gpu_out, 602112UL,
             cudaMemcpyDeviceToHost);
  c5_DeepLearningNetwork_activations(chartInstance, c5_this_Network,
    chartInstance->c5_out, c5_tmpFeatureMap);
  cudaMemcpy(chartInstance->c5_gpu_dv, &c5_dv[0], 8UL, cudaMemcpyHostToDevice);
  c5_eML_blk_kernel_kernel13<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_dv, *chartInstance->c5_gpu_anchors);
  cudaMemcpy(chartInstance->c5_gpu_dv1, &c5_dv1[0], 32UL, cudaMemcpyHostToDevice);
  c5_eML_blk_kernel_kernel14<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_dv1, *chartInstance->c5_b_gpu_anchors,
     *chartInstance->c5_gpu_anchors);
  cudaMemcpy(chartInstance->c5_gpu_tmpFeatureMap, &c5_tmpFeatureMap[0], 18816UL,
             cudaMemcpyHostToDevice);
  c5_eML_blk_kernel_kernel15<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_anchors, *chartInstance->c5_gpu_tmpFeatureMap,
     *chartInstance->c5_gpu_boxOut);
  c5_eML_blk_kernel_kernel16<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_boxOut, *chartInstance->c5_gpu_bv);
  c5_bv_dirtyOnGpu = true;
  c5_trueCount = 0;
  for (c5_i = 0; c5_i < 784; c5_i++) {
    if (c5_bv_dirtyOnGpu) {
      cudaMemcpy(&c5_bv[0], chartInstance->c5_gpu_bv, 784UL,
                 cudaMemcpyDeviceToHost);
      c5_bv_dirtyOnGpu = false;
    }

    if (c5_bv[c5_i]) {
      c5_trueCount++;
    }
  }

  c5_ii_size[0] = c5_trueCount;
  c5_partialTrueCount = 0;
  for (c5_b_i = 0; c5_b_i < 784; c5_b_i++) {
    if (c5_bv_dirtyOnGpu) {
      cudaMemcpy(&c5_bv[0], chartInstance->c5_gpu_bv, 784UL,
                 cudaMemcpyDeviceToHost);
      c5_bv_dirtyOnGpu = false;
    }

    if (c5_bv[c5_b_i]) {
      c5_ii_data[c5_partialTrueCount] = (int16_T)(c5_b_i + 1);
      c5_ii_data_dirtyOnCpu = true;
      c5_partialTrueCount++;
    }
  }

  c5_thresholdedPrediction_size[0] = c5_trueCount;
  c5_thresholdedPrediction_size[1] = 6;
  c5_thresholdedPrediction_size_dirtyOnCpu = true;
  c5_validLaunchParams = mwGetLaunchParameters((real_T)(((int64_T)(c5_ii_size[0]
    - 1) + 1L) * 6L), &c5_grid, &c5_block, 1024U, 65535U);
  if (c5_validLaunchParams) {
    if (c5_ii_data_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c5_gpu_ii_data, &c5_ii_data[0], 1568UL,
                 cudaMemcpyHostToDevice);
    }

    cudaMemcpy(chartInstance->c5_gpu_thresholdedPrediction_size,
               &c5_thresholdedPrediction_size[0], 8UL, cudaMemcpyHostToDevice);
    c5_thresholdedPrediction_size_dirtyOnCpu = false;
    cudaMemcpy(chartInstance->c5_gpu_ii_size, &c5_ii_size[0], 4UL,
               cudaMemcpyHostToDevice);
    c5_eML_blk_kernel_kernel17<<<c5_grid, c5_block>>>
      (*chartInstance->c5_gpu_boxOut, *chartInstance->c5_gpu_ii_data,
       *chartInstance->c5_gpu_thresholdedPrediction_size,
       *chartInstance->c5_gpu_ii_size,
       *chartInstance->c5_gpu_thresholdedPrediction_data);
  }

  if (c5_ii_size[0] != 0) {
    c5_i7 = c5_ii_size[0] - 1;
    c5_bboxesX1Y1X2Y2_size[0] = c5_ii_size[0];
    c5_bboxesX1Y1X2Y2_size[1] = 4;
    c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    c5_b_validLaunchParams = mwGetLaunchParameters((real_T)(((int64_T)c5_i7 + 1L)
      * 4L), &c5_b_grid, &c5_b_block, 1024U, 65535U);
    if (c5_b_validLaunchParams) {
      if (c5_thresholdedPrediction_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_thresholdedPrediction_size,
                   &c5_thresholdedPrediction_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        c5_thresholdedPrediction_size_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                 &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
      c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      c5_eML_blk_kernel_kernel18<<<c5_b_grid, c5_b_block>>>
        (*chartInstance->c5_gpu_thresholdedPrediction_data,
         *chartInstance->c5_gpu_thresholdedPrediction_size,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_i7,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
      c5_bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    c5_i10 = c5_bboxesX1Y1X2Y2_size[0] - 1;
    c5_x1_size[0] = c5_bboxesX1Y1X2Y2_size[0];
    c5_d_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i10 + 1L),
      &c5_d_grid, &c5_d_block, 1024U, 65535U);
    if (c5_d_validLaunchParams) {
      c5_eML_blk_kernel_kernel19<<<c5_d_grid, c5_d_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data, c5_i10,
         *chartInstance->c5_gpu_x1_data);
    }

    c5_i14 = c5_bboxesX1Y1X2Y2_size[0] - 1;
    c5_y1_size[0] = c5_bboxesX1Y1X2Y2_size[0];
    c5_f_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i14 + 1L),
      &c5_f_grid, &c5_f_block, 1024U, 65535U);
    if (c5_f_validLaunchParams) {
      if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel20<<<c5_f_grid, c5_f_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_i14,
         *chartInstance->c5_gpu_y1_data);
      c5_y1_data_dirtyOnGpu = true;
    }

    c5_i17 = c5_bboxesX1Y1X2Y2_size[0] - 1;
    c5_x2_size[0] = c5_bboxesX1Y1X2Y2_size[0];
    c5_g_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i17 + 1L),
      &c5_g_grid, &c5_g_block, 1024U, 65535U);
    if (c5_g_validLaunchParams) {
      if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel21<<<c5_g_grid, c5_g_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_i17,
         *chartInstance->c5_gpu_x2_data);
      c5_x2_data_dirtyOnGpu = true;
    }

    c5_i19 = c5_bboxesX1Y1X2Y2_size[0] - 1;
    c5_y2_size[0] = c5_bboxesX1Y1X2Y2_size[0];
    c5_h_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i19 + 1L),
      &c5_h_grid, &c5_h_block, 1024U, 65535U);
    if (c5_h_validLaunchParams) {
      if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
      }

      c5_eML_blk_kernel_kernel22<<<c5_h_grid, c5_h_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_i19,
         *chartInstance->c5_gpu_y2_data);
      c5_y2_data_dirtyOnGpu = true;
    }

    c5_i_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_bboxesX1Y1X2Y2_size[0] - 1) + 1L), &c5_i_grid, &c5_i_block, 1024U,
      65535U);
    if (c5_i_validLaunchParams) {
      c5_eML_blk_kernel_kernel23<<<c5_i_grid, c5_i_block>>>
        (c5_bboxesX1Y1X2Y2_size[0], *chartInstance->c5_gpu_x1_data);
    }

    c5_j_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_bboxesX1Y1X2Y2_size[0] - 1) + 1L), &c5_j_grid, &c5_j_block, 1024U,
      65535U);
    if (c5_j_validLaunchParams) {
      c5_eML_blk_kernel_kernel24<<<c5_j_grid, c5_j_block>>>
        (c5_bboxesX1Y1X2Y2_size[0], *chartInstance->c5_gpu_y1_data);
      c5_y1_data_dirtyOnGpu = true;
    }

    c5_k_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_bboxesX1Y1X2Y2_size[0] - 1) + 1L), &c5_k_grid, &c5_k_block, 1024U,
      65535U);
    if (c5_k_validLaunchParams) {
      c5_eML_blk_kernel_kernel25<<<c5_k_grid, c5_k_block>>>
        (c5_bboxesX1Y1X2Y2_size[0], *chartInstance->c5_gpu_x2_data);
      c5_x2_data_dirtyOnGpu = true;
    }

    c5_l_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_bboxesX1Y1X2Y2_size[0] - 1) + 1L), &c5_l_grid, &c5_l_block, 1024U,
      65535U);
    if (c5_l_validLaunchParams) {
      c5_eML_blk_kernel_kernel26<<<c5_l_grid, c5_l_block>>>
        (c5_bboxesX1Y1X2Y2_size[0], *chartInstance->c5_gpu_y2_data);
      c5_y2_data_dirtyOnGpu = true;
    }

    c5_bboxesX1Y1X2Y2_size[0] = c5_x1_size[0];
    c5_bboxesX1Y1X2Y2_size[1] = 4;
    c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    c5_m_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_x1_size[0] - 1) + 1L), &c5_m_grid, &c5_m_block, 1024U, 65535U);
    if (c5_m_validLaunchParams) {
      c5_eML_blk_kernel_kernel27<<<c5_m_grid, c5_m_block>>>
        (*chartInstance->c5_gpu_x1_data, c5_x1_size[0] - 1,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
      c5_bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    c5_n_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_y1_size[0] - 1) + 1L), &c5_n_grid, &c5_n_block, 1024U, 65535U);
    if (c5_n_validLaunchParams) {
      cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                 &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
      c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      c5_eML_blk_kernel_kernel28<<<c5_n_grid, c5_n_block>>>
        (*chartInstance->c5_gpu_y1_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_y1_size[0] - 1,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
      c5_bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    c5_o_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_x2_size[0] - 1) + 1L), &c5_o_grid, &c5_o_block, 1024U, 65535U);
    if (c5_o_validLaunchParams) {
      if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel29<<<c5_o_grid, c5_o_block>>>
        (*chartInstance->c5_gpu_x2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_x2_size[0] - 1,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
      c5_bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    c5_p_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_y2_size[0] - 1) + 1L), &c5_p_grid, &c5_p_block, 1024U, 65535U);
    if (c5_p_validLaunchParams) {
      if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel30<<<c5_p_grid, c5_p_block>>>
        (*chartInstance->c5_gpu_y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_y2_size[0] - 1,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
      c5_bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
    }

    c5_i25 = c5_bboxesX1Y1X2Y2_size[0];
    c5_i26 = c5_bboxesX1Y1X2Y2_size[0];
    c5_i27 = c5_bboxesX1Y1X2Y2_size[0];
    c5_bboxPred_size[0] = c5_bboxesX1Y1X2Y2_size[0];
    c5_bboxPred_size[1] = 4;
    c5_bboxPred_size_dirtyOnCpu = true;
    c5_q_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_bboxesX1Y1X2Y2_size[0] - 1) + 1L), &c5_q_grid, &c5_q_block, 1024U,
      65535U);
    if (c5_q_validLaunchParams) {
      c5_eML_blk_kernel_kernel31<<<c5_q_grid, c5_q_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data, c5_bboxesX1Y1X2Y2_size[0] -
         1, *chartInstance->c5_c_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_r_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_i25 - 1)
      + 1L), &c5_r_grid, &c5_r_block, 1024U, 65535U);
    if (c5_r_validLaunchParams) {
      if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_bboxPred_size[0],
                 8UL, cudaMemcpyHostToDevice);
      c5_bboxPred_size_dirtyOnCpu = false;
      c5_eML_blk_kernel_kernel32<<<c5_r_grid, c5_r_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
         *chartInstance->c5_d_gpu_bboxPred_size, c5_i25 - 1,
         *chartInstance->c5_c_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_s_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_i26 - 1)
      + 1L), &c5_s_grid, &c5_s_block, 1024U, 65535U);
    if (c5_s_validLaunchParams) {
      if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
      }

      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel33<<<c5_s_grid, c5_s_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
         *chartInstance->c5_d_gpu_bboxPred_size, c5_i26 - 1,
         *chartInstance->c5_c_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_t_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_i27 - 1)
      + 1L), &c5_t_grid, &c5_t_block, 1024U, 65535U);
    if (c5_t_validLaunchParams) {
      if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
      }

      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel34<<<c5_t_grid, c5_t_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
         *chartInstance->c5_d_gpu_bboxPred_size, c5_i27 - 1,
         *chartInstance->c5_c_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_nx = c5_bboxPred_size[0] << 2;
    c5_u_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_nx - 1)
      + 1L), &c5_u_grid, &c5_u_block, 1024U, 65535U);
    if (c5_u_validLaunchParams) {
      c5_eML_blk_kernel_kernel35<<<c5_u_grid, c5_u_block>>>(c5_nx,
        *chartInstance->c5_c_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_i32 = c5_bboxPred_size[0] - 1;
    c5_b_bboxPred_size[0] = c5_bboxPred_size[0];
    c5_v_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i32 + 1L),
      &c5_v_grid, &c5_v_block, 1024U, 65535U);
    if (c5_v_validLaunchParams) {
      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel36<<<c5_v_grid, c5_v_block>>>
        (*chartInstance->c5_c_gpu_bboxPred_data,
         *chartInstance->c5_d_gpu_bboxPred_size, c5_i32,
         *chartInstance->c5_gpu_bboxPred_data);
    }

    c5_w_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_b_bboxPred_size[0] - 1) + 1L), &c5_w_grid, &c5_w_block, 1024U, 65535U);
    if (c5_w_validLaunchParams) {
      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_b_bboxPred_size[0],
                 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel37<<<c5_w_grid, c5_w_block>>>
        (*chartInstance->c5_gpu_bboxPred_data,
         *chartInstance->c5_d_gpu_bboxPred_size,
         *chartInstance->c5_c_gpu_bboxPred_size,
         *chartInstance->c5_c_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_i35 = c5_bboxPred_size[0] - 1;
    c5_c_bboxPred_size[0] = c5_bboxPred_size[0];
    c5_x_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i35 + 1L),
      &c5_x_grid, &c5_x_block, 1024U, 65535U);
    if (c5_x_validLaunchParams) {
      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel38<<<c5_x_grid, c5_x_block>>>
        (*chartInstance->c5_c_gpu_bboxPred_data,
         *chartInstance->c5_d_gpu_bboxPred_size, c5_i35,
         *chartInstance->c5_d_gpu_bboxPred_data);
    }

    c5_y_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_c_bboxPred_size[0] - 1) + 1L), &c5_y_grid, &c5_y_block, 1024U, 65535U);
    if (c5_y_validLaunchParams) {
      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_b_gpu_bboxPred_size, &c5_c_bboxPred_size[0],
                 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel39<<<c5_y_grid, c5_y_block>>>
        (*chartInstance->c5_d_gpu_bboxPred_data,
         *chartInstance->c5_d_gpu_bboxPred_size,
         *chartInstance->c5_b_gpu_bboxPred_size,
         *chartInstance->c5_c_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_count = 0.0;
    c5_d_bboxPred_size[0] = c5_bboxPred_size[0];
    c5_d_bboxPred_size[1] = 4;
    c5_b_bboxPred_size_dirtyOnCpu = true;
    c5_scorePred_size[0] = c5_bboxPred_size[0];
    c5_classPred_size[0] = c5_bboxPred_size[0];
    c5_c_i = c5_bboxPred_size[0];
    for (c5_d_i = 0; c5_d_i < c5_c_i; c5_d_i++) {
      if (c5_b_bboxPred_data_dirtyOnGpu) {
        cudaMemcpy(&c5_bboxPred_data[0], chartInstance->c5_c_gpu_bboxPred_data,
                   25088UL, cudaMemcpyDeviceToHost);
        c5_b_bboxPred_data_dirtyOnGpu = false;
      }

      if ((c5_bboxPred_data[c5_d_i + c5_bboxPred_size[0] * 3] >= 1.0) &&
          (c5_bboxPred_data[c5_d_i + (c5_bboxPred_size[0] << 1)] >= 1.0) &&
          (c5_bboxPred_data[c5_d_i + c5_bboxPred_size[0] * 3] <= 480.0) &&
          (c5_bboxPred_data[c5_d_i + (c5_bboxPred_size[0] << 1)] <= 640.0)) {
        c5_count++;
        if (c5_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_bboxPred_size[0],
                     8UL, cudaMemcpyHostToDevice);
          c5_bboxPred_size_dirtyOnCpu = false;
        }

        if (c5_b_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_bboxPred_size, &c5_d_bboxPred_size[0],
                     8UL, cudaMemcpyHostToDevice);
          c5_b_bboxPred_size_dirtyOnCpu = false;
        }

        c5_eML_blk_kernel_kernel40<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*chartInstance->c5_c_gpu_bboxPred_data,
           *chartInstance->c5_d_gpu_bboxPred_size, c5_d_i,
           *chartInstance->c5_gpu_bboxPred_size, (int32_T)c5_count - 1,
           *chartInstance->c5_b_gpu_bboxPred_data);
        c5_bboxPred_data_dirtyOnGpu = true;
        if (c5_thresholdedPrediction_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_thresholdedPrediction_size,
                     &c5_thresholdedPrediction_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          c5_thresholdedPrediction_size_dirtyOnCpu = false;
        }

        c5_eML_blk_kernel_kernel41<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*chartInstance->c5_gpu_thresholdedPrediction_data,
           *chartInstance->c5_gpu_thresholdedPrediction_size, c5_d_i, c5_count, *
           chartInstance->c5_gpu_classPred_data,
           *chartInstance->c5_gpu_scorePred_data);
        c5_scorePred_data_dirtyOnGpu = true;
        c5_classPred_data_dirtyOnGpu = true;
      }
    }

    c5_i39 = c5_d_bboxPred_size[0];
    c5_ab_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_i39 -
      (int32_T)(c5_count + 1.0)) + 1L), &c5_ab_grid, &c5_ab_block, 1024U, 65535U);
    if (c5_ab_validLaunchParams) {
      c5_eML_blk_kernel_kernel42<<<c5_ab_grid, c5_ab_block>>>((int32_T)(c5_count
        + 1.0), c5_i39, *chartInstance->c5_gpu_idx_data);
      c5_idx_data_dirtyOnGpu = true;
    }

    c5_nrowx = c5_d_bboxPred_size[0];
    if ((c5_d_bboxPred_size[0] - (int32_T)(c5_count + 1.0)) + 1 == 1) {
      c5_nrows = c5_d_bboxPred_size[0] - 1;
      c5_nrows_dirtyOnGpu = false;
      if (c5_b_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxPred_size, &c5_d_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
      }

      cudaMemcpy(chartInstance->c5_gpu_nrows, &c5_nrows, 4UL,
                 cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel46<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c5_gpu_bboxPred_size, chartInstance->c5_gpu_nrows,
         *chartInstance->c5_gpu_idx_data, *chartInstance->c5_b_gpu_bboxPred_data);
    } else {
      c5_b_size[0] = 1;
      c5_b_size[1] = c5_d_bboxPred_size[0];
      c5_bb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
        (c5_d_bboxPred_size[0] - 1) + 1L), &c5_bb_grid, &c5_bb_block, 1024U,
        65535U);
      if (c5_bb_validLaunchParams) {
        if (c5_b_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_bboxPred_size, &c5_d_bboxPred_size[0],
                     8UL, cudaMemcpyHostToDevice);
          c5_b_bboxPred_size_dirtyOnCpu = false;
        }

        c5_eML_blk_kernel_kernel43<<<c5_bb_grid, c5_bb_block>>>
          (*chartInstance->c5_gpu_bboxPred_size, *chartInstance->c5_gpu_b_data);
        c5_b_data_dirtyOnGpu = true;
      }

      c5_i1 = (c5_d_bboxPred_size[0] - (int32_T)(c5_count + 1.0)) + 1;
      for (c5_c_k = 0; c5_c_k < c5_i1; c5_c_k++) {
        if (c5_idx_data_dirtyOnGpu) {
          cudaMemcpy(&c5_idx_data[0], chartInstance->c5_gpu_idx_data, 3136UL,
                     cudaMemcpyDeviceToHost);
          c5_idx_data_dirtyOnGpu = false;
        }

        if (c5_b_data_dirtyOnGpu) {
          cudaMemcpy(&c5_b_data[0], chartInstance->c5_gpu_b_data, 784UL,
                     cudaMemcpyDeviceToHost);
          c5_b_data_dirtyOnGpu = false;
        }

        c5_b_data[c5_idx_data[c5_c_k] - 1] = true;
        c5_b_data_dirtyOnCpu = true;
      }

      c5_n = 0;
      c5_n_dirtyOnCpu = true;
      c5_i3 = c5_b_size[1];
      c5_cb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
        (c5_b_size[1] - 1) + 1L), &c5_cb_grid, &c5_cb_block, 1024U, 65535U);
      if (c5_cb_validLaunchParams) {
        if (c5_b_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_b_data, &c5_b_data[0], 784UL,
                     cudaMemcpyHostToDevice);
          c5_b_data_dirtyOnCpu = false;
        }

        cudaMemcpy(chartInstance->c5_gpu_b_size, &c5_b_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        cudaMemcpy(chartInstance->c5_b_gpu_n, &c5_n, 4UL, cudaMemcpyHostToDevice);
        c5_eML_blk_kernel_kernel44<<<c5_cb_grid, c5_cb_block>>>
          (*chartInstance->c5_gpu_b_size, *chartInstance->c5_gpu_b_data, c5_i3,
           chartInstance->c5_b_gpu_n);
        c5_n_dirtyOnCpu = false;
      }

      if (c5_b_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxPred_size, &c5_d_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
      }

      if (c5_n_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_b_gpu_n, &c5_n, 4UL, cudaMemcpyHostToDevice);
      }

      c5_eML_blk_kernel_kernel45<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (chartInstance->c5_b_gpu_n, *chartInstance->c5_gpu_bboxPred_size,
         chartInstance->c5_gpu_nrows);
      c5_nrows_dirtyOnGpu = true;
      c5_e_i = 0;
      for (c5_d_k = 0; c5_d_k < c5_nrowx; c5_d_k++) {
        c5_guard1 = false;
        if (c5_d_k + 1 > c5_b_size[1]) {
          c5_guard1 = true;
        } else {
          if (c5_b_data_dirtyOnGpu) {
            cudaMemcpy(&c5_b_data[0], chartInstance->c5_gpu_b_data, 784UL,
                       cudaMemcpyDeviceToHost);
            c5_b_data_dirtyOnGpu = false;
          }

          if (!c5_b_data[c5_d_k]) {
            c5_guard1 = true;
          }
        }

        if (c5_guard1) {
          for (c5_j = 0; c5_j < 4; c5_j++) {
            if (c5_bboxPred_data_dirtyOnGpu) {
              cudaMemcpy(&c5_b_bboxPred_data[0],
                         chartInstance->c5_b_gpu_bboxPred_data, 25088UL,
                         cudaMemcpyDeviceToHost);
              c5_bboxPred_data_dirtyOnGpu = false;
            }

            c5_b_bboxPred_data[c5_e_i + c5_d_bboxPred_size[0] * c5_j] =
              c5_b_bboxPred_data[c5_d_k + c5_d_bboxPred_size[0] * c5_j];
            c5_bboxPred_data_dirtyOnCpu = true;
          }

          c5_e_i++;
        }
      }
    }

    if (c5_nrows_dirtyOnGpu) {
      cudaMemcpy(&c5_nrows, chartInstance->c5_gpu_nrows, 4UL,
                 cudaMemcpyDeviceToHost);
    }

    if (1 > c5_nrows) {
      c5_i4 = -1;
    } else {
      c5_i4 = c5_nrows - 1;
    }

    c5_e_bboxPred_size[0] = c5_i4 + 1;
    c5_e_bboxPred_size[1] = 4;
    c5_c_bboxPred_size_dirtyOnCpu = true;
    c5_db_validLaunchParams = mwGetLaunchParameters((real_T)(((int64_T)c5_i4 +
      1L) * 4L), &c5_db_grid, &c5_db_block, 1024U, 65535U);
    if (c5_db_validLaunchParams) {
      if (c5_bboxPred_data_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_b_gpu_bboxPred_data, &c5_b_bboxPred_data[0],
                   25088UL, cudaMemcpyHostToDevice);
        c5_bboxPred_data_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_e_gpu_bboxPred_size, &c5_e_bboxPred_size[0],
                 8UL, cudaMemcpyHostToDevice);
      c5_c_bboxPred_size_dirtyOnCpu = false;
      c5_eML_blk_kernel_kernel47<<<c5_db_grid, c5_db_block>>>
        (*chartInstance->c5_b_gpu_bboxPred_data,
         *chartInstance->c5_gpu_bboxPred_size,
         *chartInstance->c5_e_gpu_bboxPred_size, c5_i4,
         *chartInstance->c5_e_gpu_bboxPred_data);
    }

    c5_d_bboxPred_size[0] = c5_e_bboxPred_size[0];
    c5_d_bboxPred_size[1] = 4;
    c5_b_bboxPred_size_dirtyOnCpu = true;
    c5_eb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_e_bboxPred_size[0] * 4 - 1) + 1L), &c5_eb_grid, &c5_eb_block, 1024U,
      65535U);
    if (c5_eb_validLaunchParams) {
      if (c5_c_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_e_gpu_bboxPred_size, &c5_e_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
      }

      c5_eML_blk_kernel_kernel48<<<c5_eb_grid, c5_eb_block>>>
        (*chartInstance->c5_e_gpu_bboxPred_data,
         *chartInstance->c5_e_gpu_bboxPred_size,
         *chartInstance->c5_b_gpu_bboxPred_data);
      c5_bboxPred_data_dirtyOnCpu = false;
    }

    c5_i47 = c5_scorePred_size[0];
    c5_fb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_i47 -
      (int32_T)(c5_count + 1.0)) + 1L), &c5_fb_grid, &c5_fb_block, 1024U, 65535U);
    if (c5_fb_validLaunchParams) {
      c5_eML_blk_kernel_kernel49<<<c5_fb_grid, c5_fb_block>>>((int32_T)(c5_count
        + 1.0), c5_i47, *chartInstance->c5_gpu_idx_data);
      c5_idx_data_dirtyOnGpu = true;
    }

    c5_nxin = c5_scorePred_size[0];
    c5_b_size[0] = 1;
    c5_b_size[1] = c5_scorePred_size[0];
    c5_gb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_scorePred_size[0] - 1) + 1L), &c5_gb_grid, &c5_gb_block, 1024U, 65535U);
    if (c5_gb_validLaunchParams) {
      cudaMemcpy(chartInstance->c5_gpu_scorePred_size, &c5_scorePred_size[0],
                 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel50<<<c5_gb_grid, c5_gb_block>>>
        (*chartInstance->c5_gpu_scorePred_size, *chartInstance->c5_gpu_b_data);
      c5_b_data_dirtyOnCpu = false;
      c5_b_data_dirtyOnGpu = true;
    }

    c5_i5 = (c5_scorePred_size[0] - (int32_T)(c5_count + 1.0)) + 1;
    for (c5_e_k = 0; c5_e_k < c5_i5; c5_e_k++) {
      if (c5_idx_data_dirtyOnGpu) {
        cudaMemcpy(&c5_idx_data[0], chartInstance->c5_gpu_idx_data, 3136UL,
                   cudaMemcpyDeviceToHost);
        c5_idx_data_dirtyOnGpu = false;
      }

      if (c5_b_data_dirtyOnGpu) {
        cudaMemcpy(&c5_b_data[0], chartInstance->c5_gpu_b_data, 784UL,
                   cudaMemcpyDeviceToHost);
        c5_b_data_dirtyOnGpu = false;
      }

      c5_b_data[c5_idx_data[c5_e_k] - 1] = true;
      c5_b_data_dirtyOnCpu = true;
    }

    c5_b_n = 0;
    c5_n_dirtyOnGpu = false;
    c5_i6 = c5_b_size[1];
    c5_hb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_b_size[1] - 1) + 1L), &c5_hb_grid, &c5_hb_block, 1024U, 65535U);
    if (c5_hb_validLaunchParams) {
      if (c5_b_data_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_b_data, &c5_b_data[0], 784UL,
                   cudaMemcpyHostToDevice);
        c5_b_data_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_gpu_b_size, &c5_b_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      cudaMemcpy(chartInstance->c5_gpu_n, &c5_b_n, 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel51<<<c5_hb_grid, c5_hb_block>>>
        (*chartInstance->c5_gpu_b_size, *chartInstance->c5_gpu_b_data, c5_i6,
         chartInstance->c5_gpu_n);
      c5_n_dirtyOnGpu = true;
    }

    if (c5_n_dirtyOnGpu) {
      cudaMemcpy(&c5_b_n, chartInstance->c5_gpu_n, 4UL, cudaMemcpyDeviceToHost);
    }

    c5_nxout = c5_scorePred_size[0] - c5_b_n;
    c5_k0 = -1;
    for (c5_f_k = 0; c5_f_k < c5_nxin; c5_f_k++) {
      c5_guard1 = false;
      if (c5_f_k + 1 > c5_b_size[1]) {
        c5_guard1 = true;
      } else {
        if (c5_b_data_dirtyOnGpu) {
          cudaMemcpy(&c5_b_data[0], chartInstance->c5_gpu_b_data, 784UL,
                     cudaMemcpyDeviceToHost);
          c5_b_data_dirtyOnGpu = false;
        }

        if (!c5_b_data[c5_f_k]) {
          c5_guard1 = true;
        }
      }

      if (c5_guard1) {
        c5_k0++;
        if (c5_scorePred_data_dirtyOnGpu) {
          cudaMemcpy(&c5_scorePred_data[0], chartInstance->c5_gpu_scorePred_data,
                     3136UL, cudaMemcpyDeviceToHost);
          c5_scorePred_data_dirtyOnGpu = false;
        }

        c5_scorePred_data[c5_k0] = c5_scorePred_data[c5_f_k];
        c5_scorePred_data_dirtyOnCpu = true;
      }
    }

    if (1 > c5_nxout) {
      c5_b_i7 = 0;
    } else {
      c5_b_i7 = c5_nxout;
    }

    c5_scorePred_size[0] = c5_b_i7;
    c5_i51 = c5_classPred_size[0];
    c5_ib_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_i51 -
      (int32_T)(c5_count + 1.0)) + 1L), &c5_ib_grid, &c5_ib_block, 1024U, 65535U);
    if (c5_ib_validLaunchParams) {
      c5_eML_blk_kernel_kernel52<<<c5_ib_grid, c5_ib_block>>>((int32_T)(c5_count
        + 1.0), c5_i51, *chartInstance->c5_gpu_idx_data);
      c5_idx_data_dirtyOnGpu = true;
    }

    c5_b_nxin = c5_classPred_size[0];
    c5_b_size[0] = 1;
    c5_b_size[1] = c5_classPred_size[0];
    c5_jb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_classPred_size[0] - 1) + 1L), &c5_jb_grid, &c5_jb_block, 1024U, 65535U);
    if (c5_jb_validLaunchParams) {
      cudaMemcpy(chartInstance->c5_gpu_classPred_size, &c5_classPred_size[0],
                 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel53<<<c5_jb_grid, c5_jb_block>>>
        (*chartInstance->c5_gpu_classPred_size, *chartInstance->c5_gpu_b_data);
      c5_b_data_dirtyOnCpu = false;
      c5_b_data_dirtyOnGpu = true;
    }

    c5_i8 = (c5_classPred_size[0] - (int32_T)(c5_count + 1.0)) + 1;
    for (c5_g_k = 0; c5_g_k < c5_i8; c5_g_k++) {
      if (c5_idx_data_dirtyOnGpu) {
        cudaMemcpy(&c5_idx_data[0], chartInstance->c5_gpu_idx_data, 3136UL,
                   cudaMemcpyDeviceToHost);
        c5_idx_data_dirtyOnGpu = false;
      }

      if (c5_b_data_dirtyOnGpu) {
        cudaMemcpy(&c5_b_data[0], chartInstance->c5_gpu_b_data, 784UL,
                   cudaMemcpyDeviceToHost);
        c5_b_data_dirtyOnGpu = false;
      }

      c5_b_data[c5_idx_data[c5_g_k] - 1] = true;
      c5_b_data_dirtyOnCpu = true;
    }

    c5_c_n = 0;
    c5_b_n_dirtyOnGpu = false;
    c5_i9 = c5_b_size[1];
    c5_kb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_b_size[1] - 1) + 1L), &c5_kb_grid, &c5_kb_block, 1024U, 65535U);
    if (c5_kb_validLaunchParams) {
      if (c5_b_data_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_b_data, &c5_b_data[0], 784UL,
                   cudaMemcpyHostToDevice);
      }

      cudaMemcpy(chartInstance->c5_gpu_b_size, &c5_b_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      cudaMemcpy(chartInstance->c5_c_gpu_n, &c5_c_n, 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel54<<<c5_kb_grid, c5_kb_block>>>
        (*chartInstance->c5_gpu_b_size, *chartInstance->c5_gpu_b_data, c5_i9,
         chartInstance->c5_c_gpu_n);
      c5_b_n_dirtyOnGpu = true;
    }

    if (c5_b_n_dirtyOnGpu) {
      cudaMemcpy(&c5_c_n, chartInstance->c5_c_gpu_n, 4UL, cudaMemcpyDeviceToHost);
    }

    c5_b_k0 = -1;
    for (c5_h_k = 0; c5_h_k < c5_b_nxin; c5_h_k++) {
      c5_guard1 = false;
      if (c5_h_k + 1 > c5_b_size[1]) {
        c5_guard1 = true;
      } else {
        if (c5_b_data_dirtyOnGpu) {
          cudaMemcpy(&c5_b_data[0], chartInstance->c5_gpu_b_data, 784UL,
                     cudaMemcpyDeviceToHost);
          c5_b_data_dirtyOnGpu = false;
        }

        if (!c5_b_data[c5_h_k]) {
          c5_guard1 = true;
        }
      }

      if (c5_guard1) {
        c5_b_k0++;
        if (c5_classPred_data_dirtyOnGpu) {
          cudaMemcpy(&c5_classPred_data[0], chartInstance->c5_gpu_classPred_data,
                     3136UL, cudaMemcpyDeviceToHost);
          c5_classPred_data_dirtyOnGpu = false;
        }

        c5_classPred_data[c5_b_k0] = c5_classPred_data[c5_h_k];
        c5_classPred_data_dirtyOnCpu = true;
      }
    }

    if (c5_d_bboxPred_size[0] == 0) {
      c5_bboxesX1Y1X2Y2_size[0] = 0;
      c5_bboxesX1Y1X2Y2_size[1] = 4;
      c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
      c5_b_scores_size[0] = c5_scorePred_size[0];
      c5_lb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
        (c5_scorePred_size[0] - 1) + 1L), &c5_lb_grid, &c5_lb_block, 1024U,
        65535U);
      if (c5_lb_validLaunchParams) {
        if (c5_scorePred_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_scorePred_data, &c5_scorePred_data[0],
                     3136UL, cudaMemcpyHostToDevice);
        }

        cudaMemcpy(chartInstance->c5_gpu_scorePred_size, &c5_scorePred_size[0],
                   4UL, cudaMemcpyHostToDevice);
        c5_eML_blk_kernel_kernel66<<<c5_lb_grid, c5_lb_block>>>
          (*chartInstance->c5_gpu_scorePred_data,
           *chartInstance->c5_gpu_scorePred_size,
           *chartInstance->c5_b_gpu_scores_data);
      }
    } else {
      c5_x1_size[0] = c5_b_i7;
      c5_x1_size_dirtyOnCpu = true;
      if (c5_b_i7 != 0) {
        c5_sortDim = 2;
        if (c5_b_i7 != 1) {
          c5_sortDim = 1;
        }

        c5_inDims[0] = c5_b_i7;
        c5_inDims[1] = 1;
        c5_nb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
          (c5_scorePred_size[0] - 1) + 1L), &c5_nb_grid, &c5_nb_block, 1024U,
          65535U);
        if (c5_nb_validLaunchParams) {
          if (c5_scorePred_data_dirtyOnCpu) {
            cudaMemcpy(chartInstance->c5_gpu_scorePred_data, &c5_scorePred_data
                       [0], 3136UL, cudaMemcpyHostToDevice);
          }

          cudaMemcpy(chartInstance->c5_gpu_scorePred_size, &c5_scorePred_size[0],
                     4UL, cudaMemcpyHostToDevice);
          c5_eML_blk_kernel_kernel55<<<c5_nb_grid, c5_nb_block>>>
            (*chartInstance->c5_gpu_scorePred_data,
             *chartInstance->c5_gpu_scorePred_size,
             *chartInstance->c5_gpu_out_data);
        }

        c5_dv2[0] = (uint32_T)c5_b_i7;
        c5_x1_size[0] = c5_b_i7;
        c5_pb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
          ((int32_T)c5_dv2[0] - 1) + 1L), &c5_pb_grid, &c5_pb_block, 1024U,
          65535U);
        if (c5_pb_validLaunchParams) {
          cudaMemcpy(chartInstance->c5_gpu_dv2, &c5_dv2[0], 8UL,
                     cudaMemcpyHostToDevice);
          c5_eML_blk_kernel_kernel56<<<c5_pb_grid, c5_pb_block>>>
            (*chartInstance->c5_gpu_dv2, *chartInstance->c5_gpu_x1_data);
        }

        thrustSortImplWithIndex(&(*chartInstance->c5_gpu_out_data)[0],
          &(*chartInstance->c5_gpu_x1_data)[0], 2, &c5_inDims[0], c5_sortDim,
          'd', false);
      }

      c5_bboxesX1Y1X2Y2_size[0] = c5_x1_size[0];
      c5_bboxesX1Y1X2Y2_size[1] = 4;
      c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
      c5_mb_validLaunchParams = mwGetLaunchParameters((real_T)(((int64_T)
        (c5_x1_size[0] - 1) + 1L) * 4L), &c5_mb_grid, &c5_mb_block, 1024U,
        65535U);
      if (c5_mb_validLaunchParams) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        cudaMemcpy(chartInstance->c5_gpu_bboxPred_size, &c5_d_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_b_bboxPred_size_dirtyOnCpu = false;
        if (c5_bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_b_gpu_bboxPred_data, &c5_b_bboxPred_data
                     [0], 25088UL, cudaMemcpyHostToDevice);
          c5_bboxPred_data_dirtyOnCpu = false;
        }

        cudaMemcpy(chartInstance->c5_gpu_x1_size, &c5_x1_size[0], 4UL,
                   cudaMemcpyHostToDevice);
        c5_x1_size_dirtyOnCpu = false;
        c5_eML_blk_kernel_kernel57<<<c5_mb_grid, c5_mb_block>>>
          (*chartInstance->c5_b_gpu_bboxPred_data,
           *chartInstance->c5_gpu_bboxPred_size, *chartInstance->c5_gpu_x1_data,
           *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
           *chartInstance->c5_gpu_x1_size,
           *chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
        c5_bboxesX1Y1X2Y2_data_dirtyOnGpu = true;
      }

      c5_ob_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
        (c5_x1_size[0] - 1) + 1L), &c5_ob_grid, &c5_ob_block, 1024U, 65535U);
      if (c5_ob_validLaunchParams) {
        if (c5_classPred_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_classPred_data, &c5_classPred_data[0],
                     3136UL, cudaMemcpyHostToDevice);
        }

        if (c5_x1_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_x1_size, &c5_x1_size[0], 4UL,
                     cudaMemcpyHostToDevice);
          c5_x1_size_dirtyOnCpu = false;
        }

        c5_eML_blk_kernel_kernel58<<<c5_ob_grid, c5_ob_block>>>
          (*chartInstance->c5_gpu_classPred_data, *chartInstance->c5_gpu_x1_data,
           *chartInstance->c5_gpu_x1_size, *chartInstance->c5_gpu_y1_data);
        c5_y1_data_dirtyOnGpu = true;
      }

      c5_selectedIndex_size[0] = c5_x1_size[0];
      c5_qb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
        (c5_x1_size[0] - 1) + 1L), &c5_qb_grid, &c5_qb_block, 1024U, 65535U);
      if (c5_qb_validLaunchParams) {
        if (c5_x1_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_x1_size, &c5_x1_size[0], 4UL,
                     cudaMemcpyHostToDevice);
        }

        c5_eML_blk_kernel_kernel59<<<c5_qb_grid, c5_qb_block>>>
          (*chartInstance->c5_gpu_x1_size,
           *chartInstance->c5_gpu_selectedIndex_data);
        c5_selectedIndex_data_dirtyOnGpu = true;
      }

      c5_i61 = c5_x1_size[0] - 1;
      c5_rb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i61 +
        1L), &c5_rb_grid, &c5_rb_block, 1024U, 65535U);
      if (c5_rb_validLaunchParams) {
        if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                     &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
          c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        }

        c5_eML_blk_kernel_kernel60<<<c5_rb_grid, c5_rb_block>>>
          (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
           *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_i61,
           *chartInstance->c5_gpu_area_data);
        c5_area_data_dirtyOnGpu = true;
      }

      c5_i64 = c5_x1_size[0] - 1;
      c5_sb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i64 +
        1L), &c5_sb_grid, &c5_sb_block, 1024U, 65535U);
      if (c5_sb_validLaunchParams) {
        if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                     &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
          c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        }

        c5_eML_blk_kernel_kernel61<<<c5_sb_grid, c5_sb_block>>>
          (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
           *chartInstance->c5_gpu_bboxesX1Y1X2Y2_data, c5_i64,
           *chartInstance->c5_gpu_x2_data);
        c5_x2_data_dirtyOnGpu = true;
      }

      c5_i66 = c5_x1_size[0] - 1;
      c5_tb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i66 +
        1L), &c5_tb_grid, &c5_tb_block, 1024U, 65535U);
      if (c5_tb_validLaunchParams) {
        if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                     &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        }

        c5_eML_blk_kernel_kernel62<<<c5_tb_grid, c5_tb_block>>>
          (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
           *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, c5_i66,
           *chartInstance->c5_gpu_y2_data);
        c5_y2_data_dirtyOnGpu = true;
      }

      c5_currentBox = -1;
      c5_i11 = c5_x1_size[0];
      for (c5_f_i = 0; c5_f_i < c5_i11; c5_f_i++) {
        c5_currentBox = c5_f_i;
        if (c5_y1_data_dirtyOnGpu) {
          cudaMemcpy(&c5_y1_data[0], chartInstance->c5_gpu_y1_data, 6272UL,
                     cudaMemcpyDeviceToHost);
          c5_y1_data_dirtyOnGpu = false;
        }

        if (muDoubleScalarIsNaN(c5_y1_data[c5_f_i])) {
          if (c5_selectedIndex_data_dirtyOnGpu) {
            cudaMemcpy(&c5_selectedIndex_data[0],
                       chartInstance->c5_gpu_selectedIndex_data, 784UL,
                       cudaMemcpyDeviceToHost);
            c5_selectedIndex_data_dirtyOnGpu = false;
          }

          c5_selectedIndex_data[c5_f_i] = false;
          c5_selectedIndex_data_dirtyOnCpu = true;
        } else {
          if (c5_selectedIndex_data_dirtyOnGpu) {
            cudaMemcpy(&c5_selectedIndex_data[0],
                       chartInstance->c5_gpu_selectedIndex_data, 784UL,
                       cudaMemcpyDeviceToHost);
            c5_selectedIndex_data_dirtyOnGpu = false;
          }

          if (c5_selectedIndex_data[c5_f_i]) {
            c5_b_i14 = (c5_x1_size[0] - c5_f_i) - 2;
            for (c5_b_j = 0; c5_b_j <= c5_b_i14; c5_b_j++) {
              c5_c_j = (c5_f_i + c5_b_j) + 1;
              if (c5_selectedIndex_data[c5_c_j] && (!(c5_y1_data[c5_c_j] !=
                    c5_y1_data[c5_f_i]))) {
                if (c5_bboxesX1Y1X2Y2_data_dirtyOnGpu) {
                  cudaMemcpy(&c5_bboxesX1Y1X2Y2_data[0],
                             chartInstance->c5_gpu_bboxesX1Y1X2Y2_data, 25088UL,
                             cudaMemcpyDeviceToHost);
                  c5_bboxesX1Y1X2Y2_data_dirtyOnGpu = false;
                }

                if (c5_x2_data_dirtyOnGpu) {
                  cudaMemcpy(&c5_x2_data[0], chartInstance->c5_gpu_x2_data,
                             6272UL, cudaMemcpyDeviceToHost);
                  c5_x2_data_dirtyOnGpu = false;
                }

                c5_width = muDoubleScalarMin(c5_x2_data[c5_f_i],
                  c5_x2_data[c5_c_j]) - muDoubleScalarMax
                  (c5_bboxesX1Y1X2Y2_data[c5_f_i], c5_bboxesX1Y1X2Y2_data[c5_c_j]);
                if (!(c5_width <= 0.0)) {
                  if (c5_y2_data_dirtyOnGpu) {
                    cudaMemcpy(&c5_y2_data[0], chartInstance->c5_gpu_y2_data,
                               6272UL, cudaMemcpyDeviceToHost);
                    c5_y2_data_dirtyOnGpu = false;
                  }

                  c5_height = muDoubleScalarMin(c5_y2_data[c5_f_i],
                    c5_y2_data[c5_c_j]) - muDoubleScalarMax
                    (c5_bboxesX1Y1X2Y2_data[c5_f_i + c5_bboxesX1Y1X2Y2_size[0]],
                     c5_bboxesX1Y1X2Y2_data[c5_c_j + c5_bboxesX1Y1X2Y2_size[0]]);
                  if (!(c5_height <= 0.0)) {
                    c5_areaOfIntersect = c5_width * c5_height;
                    if (c5_area_data_dirtyOnGpu) {
                      cudaMemcpy(&c5_area_data[0],
                                 chartInstance->c5_gpu_area_data, 6272UL,
                                 cudaMemcpyDeviceToHost);
                      c5_area_data_dirtyOnGpu = false;
                    }

                    if (c5_areaOfIntersect / ((c5_area_data[c5_f_i] +
                          c5_area_data[c5_c_j]) - c5_areaOfIntersect) > 0.5) {
                      c5_selectedIndex_data[c5_c_j] = false;
                      c5_selectedIndex_data_dirtyOnCpu = true;
                    }
                  }
                }
              }
            }
          }
        }
      }

      if (c5_currentBox + 2 > c5_selectedIndex_size[0]) {
        c5_i12 = 0;
        c5_i13 = 0;
      } else {
        c5_i12 = c5_currentBox + 1;
        c5_i13 = c5_selectedIndex_size[0];
      }

      c5_iv[1] = c5_i13 - c5_i12;
      c5_ub_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_iv[1]
        - 1) + 1L), &c5_ub_grid, &c5_ub_block, 1024U, 65535U);
      if (c5_ub_validLaunchParams) {
        if (c5_selectedIndex_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_selectedIndex_data,
                     &c5_selectedIndex_data[0], 784UL, cudaMemcpyHostToDevice);
          c5_selectedIndex_data_dirtyOnCpu = false;
        }

        cudaMemcpy(chartInstance->c5_gpu_iv, &c5_iv[0], 8UL,
                   cudaMemcpyHostToDevice);
        c5_eML_blk_kernel_kernel63<<<c5_ub_grid, c5_ub_block>>>(c5_i12,
          *chartInstance->c5_gpu_iv, *chartInstance->c5_gpu_selectedIndex_data);
      }

      c5_index_size[0] = c5_selectedIndex_size[0];
      c5_vb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
        (c5_selectedIndex_size[0] - 1) + 1L), &c5_vb_grid, &c5_vb_block, 1024U,
        65535U);
      if (c5_vb_validLaunchParams) {
        if (c5_selectedIndex_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_selectedIndex_data,
                     &c5_selectedIndex_data[0], 784UL, cudaMemcpyHostToDevice);
        }

        cudaMemcpy(chartInstance->c5_gpu_selectedIndex_size,
                   &c5_selectedIndex_size[0], 4UL, cudaMemcpyHostToDevice);
        c5_eML_blk_kernel_kernel64<<<c5_vb_grid, c5_vb_block>>>
          (*chartInstance->c5_gpu_selectedIndex_data,
           *chartInstance->c5_gpu_x1_data,
           *chartInstance->c5_gpu_selectedIndex_size,
           *chartInstance->c5_gpu_index_data);
        c5_index_data_dirtyOnGpu = true;
      }

      c5_end = c5_index_size[0] - 1;
      c5_b_trueCount = 0;
      for (c5_g_i = 0; c5_g_i <= c5_end; c5_g_i++) {
        if (c5_index_data_dirtyOnGpu) {
          cudaMemcpy(&c5_index_data[0], chartInstance->c5_gpu_index_data, 784UL,
                     cudaMemcpyDeviceToHost);
          c5_index_data_dirtyOnGpu = false;
        }

        if (c5_index_data[c5_g_i]) {
          c5_b_trueCount++;
        }
      }

      c5_iv1_size[0] = c5_b_trueCount;
      c5_b_partialTrueCount = 0;
      for (c5_h_i = 0; c5_h_i <= c5_end; c5_h_i++) {
        if (c5_index_data_dirtyOnGpu) {
          cudaMemcpy(&c5_index_data[0], chartInstance->c5_gpu_index_data, 784UL,
                     cudaMemcpyDeviceToHost);
          c5_index_data_dirtyOnGpu = false;
        }

        if (c5_index_data[c5_h_i]) {
          c5_iv1_data[c5_b_partialTrueCount] = (int16_T)(c5_h_i + 1);
          c5_iv1_data_dirtyOnCpu = true;
          c5_b_partialTrueCount++;
        }
      }

      c5_bboxesX1Y1X2Y2_size[0] = c5_b_trueCount;
      c5_bboxesX1Y1X2Y2_size[1] = 4;
      c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
      c5_wb_validLaunchParams = mwGetLaunchParameters((real_T)(((int64_T)
        (c5_iv1_size[0] - 1) + 1L) * 4L), &c5_wb_grid, &c5_wb_block, 1024U,
        65535U);
      if (c5_wb_validLaunchParams) {
        cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                   &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
        c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = false;
        if (c5_b_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_bboxPred_size, &c5_d_bboxPred_size[0],
                     8UL, cudaMemcpyHostToDevice);
        }

        if (c5_bboxPred_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_b_gpu_bboxPred_data, &c5_b_bboxPred_data
                     [0], 25088UL, cudaMemcpyHostToDevice);
        }

        if (c5_iv1_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_iv1_data, &c5_iv1_data[0], 1568UL,
                     cudaMemcpyHostToDevice);
        }

        cudaMemcpy(chartInstance->c5_gpu_iv1_size, &c5_iv1_size[0], 4UL,
                   cudaMemcpyHostToDevice);
        c5_eML_blk_kernel_kernel65<<<c5_wb_grid, c5_wb_block>>>
          (*chartInstance->c5_b_gpu_bboxPred_data,
           *chartInstance->c5_gpu_bboxPred_size, *chartInstance->c5_gpu_iv1_data,
           *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
           *chartInstance->c5_gpu_iv1_size,
           *chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
      }

      c5_b_end = c5_index_size[0] - 1;
      c5_c_trueCount = 0;
      for (c5_i_i = 0; c5_i_i <= c5_b_end; c5_i_i++) {
        if (c5_index_data_dirtyOnGpu) {
          cudaMemcpy(&c5_index_data[0], chartInstance->c5_gpu_index_data, 784UL,
                     cudaMemcpyDeviceToHost);
          c5_index_data_dirtyOnGpu = false;
        }

        if (c5_index_data[c5_i_i]) {
          c5_c_trueCount++;
        }
      }

      c5_b_scores_size[0] = c5_c_trueCount;
      c5_c_partialTrueCount = 0;
      for (c5_j_i = 0; c5_j_i <= c5_b_end; c5_j_i++) {
        if (c5_index_data_dirtyOnGpu) {
          cudaMemcpy(&c5_index_data[0], chartInstance->c5_gpu_index_data, 784UL,
                     cudaMemcpyDeviceToHost);
          c5_index_data_dirtyOnGpu = false;
        }

        if (c5_index_data[c5_j_i]) {
          if (c5_scorePred_data_dirtyOnGpu) {
            cudaMemcpy(&c5_scorePred_data[0],
                       chartInstance->c5_gpu_scorePred_data, 3136UL,
                       cudaMemcpyDeviceToHost);
            c5_scorePred_data_dirtyOnGpu = false;
          }

          c5_c_scores_data[c5_c_partialTrueCount] = c5_scorePred_data[c5_j_i];
          c5_scores_data_dirtyOnCpu = true;
          c5_c_partialTrueCount++;
        }
      }

      c5_b_nx = c5_index_size[0];
      c5_idx = 0;
      c5_ii = 1;
      c5_exitg1 = false;
      while ((!c5_exitg1) && (c5_ii <= c5_b_nx)) {
        if (c5_index_data_dirtyOnGpu) {
          cudaMemcpy(&c5_index_data[0], chartInstance->c5_gpu_index_data, 784UL,
                     cudaMemcpyDeviceToHost);
          c5_index_data_dirtyOnGpu = false;
        }

        if (c5_index_data[c5_ii - 1]) {
          c5_idx++;
          if (c5_idx >= c5_b_nx) {
            c5_exitg1 = true;
          } else {
            c5_ii++;
          }
        } else {
          c5_ii++;
        }
      }

      if (c5_index_size[0] != 1) {
        c5_iv1[0] = 1;
        if (1 > c5_idx) {
          c5_iv1[1] = 0;
        } else {
          c5_iv1[1] = c5_idx;
        }

        c5_indexShapeCheck(chartInstance, c5_index_size[0], c5_iv1);
      }
    }
  } else {
    c5_bboxesX1Y1X2Y2_size[0] = 0;
    c5_bboxesX1Y1X2Y2_size[1] = 4;
    c5_bboxesX1Y1X2Y2_size_dirtyOnCpu = true;
    c5_b_scores_size[0] = 0;
  }

  c5_bboxes_size[0] = c5_bboxesX1Y1X2Y2_size[0];
  c5_bboxes_size[1] = 4;
  c5_c_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
    (c5_bboxesX1Y1X2Y2_size[0] * 4 - 1) + 1L), &c5_c_grid, &c5_c_block, 1024U,
    65535U);
  if (c5_c_validLaunchParams) {
    if (c5_bboxesX1Y1X2Y2_size_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
                 &c5_bboxesX1Y1X2Y2_size[0], 8UL, cudaMemcpyHostToDevice);
    }

    c5_eML_blk_kernel_kernel67<<<c5_c_grid, c5_c_block>>>
      (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
       *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
       chartInstance->c5_gpu_bboxes_data);
    c5_bboxes_data_dirtyOnGpu = true;
  }

  c5_scores_size[0] = c5_b_scores_size[0];
  c5_scores_size[1] = 1;
  c5_e_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
    (c5_b_scores_size[0] - 1) + 1L), &c5_e_grid, &c5_e_block, 1024U, 65535U);
  if (c5_e_validLaunchParams) {
    if (c5_scores_data_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c5_b_gpu_scores_data, &c5_c_scores_data[0],
                 3136UL, cudaMemcpyHostToDevice);
    }

    cudaMemcpy(chartInstance->c5_gpu_scores_size, &c5_b_scores_size[0], 4UL,
               cudaMemcpyHostToDevice);
    c5_eML_blk_kernel_kernel68<<<c5_e_grid, c5_e_block>>>
      (*chartInstance->c5_b_gpu_scores_data, *chartInstance->c5_gpu_scores_size,
       chartInstance->c5_gpu_scores_data);
    c5_scores_data_dirtyOnGpu = true;
  }

  if (c5_bboxes_data_dirtyOnGpu) {
    cudaMemcpy(&c5_b_bboxes_data[0], chartInstance->c5_gpu_bboxes_data,
               (uint32_T)(c5_bboxes_size[0] * 4) * sizeof(real_T),
               cudaMemcpyDeviceToHost);
  }

  if (c5_scores_data_dirtyOnGpu) {
    cudaMemcpy(&c5_b_scores_data[0], chartInstance->c5_gpu_scores_data,
               (uint32_T)c5_scores_size[0] * sizeof(real32_T),
               cudaMemcpyDeviceToHost);
  }
}

static void c5_DeepLearningNetwork_setup(SFc5_LaneDetectionInstanceStruct
  *chartInstance, c5_yolov2ResNet50VehicleExample0_LaneDetection0 *c5_obj)
{
  c5_obj->setup();
}

static real32_T c5_callFcn(SFc5_LaneDetectionInstanceStruct *chartInstance,
  real32_T c5_input1, real32_T c5_input2)
{
  return muSingleScalarMin(c5_input1, c5_input2);
}

static real32_T c5_b_callFcn(SFc5_LaneDetectionInstanceStruct *chartInstance,
  real32_T c5_input1, real32_T c5_input2)
{
  return muSingleScalarMax(c5_input1, c5_input2);
}

static void c5_DeepLearningNetwork_activations(SFc5_LaneDetectionInstanceStruct *
  chartInstance, c5_yolov2ResNet50VehicleExample0_LaneDetection0 *c5_obj,
  real32_T c5_varargin_1[150528], real32_T c5_b_out[4704])
{
  c5_cell_wrap_18 (*c5_gpu_miniBatchT)[1];
  c5_cell_wrap_18 *c5_gpu_r;
  real32_T (*c5_gpu_varargin_1)[150528];
  real32_T (*c5_c_gpu_out)[4704];
  real32_T (*c5_gpu_outMiniBatch)[4704];
  cudaMalloc(&c5_c_gpu_out, 18816UL);
  cudaMalloc(&c5_gpu_outMiniBatch, 18816UL);
  cudaMalloc(&c5_gpu_miniBatchT, 602112UL);
  cudaMalloc(&c5_gpu_r, 602112UL);
  cudaMalloc(&c5_gpu_varargin_1, 602112UL);
  cudaMemcpy(c5_gpu_varargin_1, &c5_varargin_1[0], 602112UL,
             cudaMemcpyHostToDevice);
  c5_DeepLearningNetwork_activations_kernel69<<<dim3(294U, 1U, 1U), dim3(512U,
    1U, 1U)>>>(*c5_gpu_varargin_1, c5_gpu_r);
  c5_DeepLearningNetwork_activations_kernel70<<<dim3(294U, 1U, 1U), dim3(512U,
    1U, 1U)>>>(c5_gpu_r, *c5_gpu_miniBatchT);
  cudaMemcpy(c5_obj->getInputDataPointer(0), (*c5_gpu_miniBatchT)[0].f1,
             c5_obj->layers[0]->getOutputTensor(0)->getNumElements() * sizeof
             (real32_T), cudaMemcpyDeviceToDevice);
  c5_obj->activations(56);
  cudaMemcpy(*c5_gpu_outMiniBatch, c5_obj->getLayerOutput(56, 0), c5_obj->
             layers[56]->getOutputTensor(0)->getNumElements() * sizeof(real32_T),
             cudaMemcpyDeviceToDevice);
  c5_DeepLearningNetwork_activations_kernel71<<<dim3(10U, 1U, 1U), dim3(512U, 1U,
    1U)>>>(*c5_gpu_outMiniBatch, *c5_c_gpu_out);
  cudaMemcpy(&c5_b_out[0], c5_c_gpu_out, 18816UL, cudaMemcpyDeviceToHost);
  cudaFree(*c5_gpu_varargin_1);
  cudaFree(c5_gpu_r);
  cudaFree(*c5_gpu_miniBatchT);
  cudaFree(*c5_gpu_outMiniBatch);
  cudaFree(*c5_c_gpu_out);
}

static void c5_indexShapeCheck(SFc5_LaneDetectionInstanceStruct *chartInstance,
  int32_T c5_matrixSize, int32_T c5_indexSize[2])
{
}

static void c5_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct *chartInstance,
  const mxArray *c5_bboxes, const char_T *c5_identifier, real_T c5_y_data[],
  int32_T c5_y_size[2])
{
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = const_cast<const char_T *>(c5_identifier);
  c5_thisId.fParent = NULL;
  c5_thisId.bParentIsCell = false;
  c5_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_bboxes), &c5_thisId,
                        c5_y_data, c5_y_size);
  sf_mex_destroy(&c5_bboxes);
}

static void c5_b_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real_T c5_y_data[], int32_T c5_y_size[2])
{
  real_T c5_dv_data[80];
  int32_T c5_dv_size[2];
  int32_T c5_i;
  int32_T c5_i1;
  int32_T c5_i2;
  uint32_T c5_uv[2];
  boolean_T c5_bv[2];
  for (c5_i = 0; c5_i < 2; c5_i++) {
    c5_uv[c5_i] = (uint32_T)(-16 * c5_i) + 20U;
  }

  c5_dv_size[0] = sf_mex_get_dimension(c5_u, 0);
  c5_dv_size[1] = sf_mex_get_dimension(c5_u, 1);
  for (c5_i1 = 0; c5_i1 < 2; c5_i1++) {
    c5_bv[c5_i1] = true;
  }

  sf_mex_import_vs(c5_parentId, sf_mex_dup(c5_u), &c5_dv_data, 1, 0, 0U, 1, 0U,
                   2, c5_bv, c5_uv, c5_dv_size);
  c5_y_size[0] = c5_dv_size[0];
  c5_y_size[1] = c5_dv_size[1];
  for (c5_i2 = 0; c5_i2 < c5_dv_size[0] * c5_dv_size[1]; c5_i2++) {
    c5_y_data[c5_i2] = c5_dv_data[c5_i2];
  }

  sf_mex_destroy(&c5_u);
}

static void c5_c_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_scores, const char_T *c5_identifier,
  real32_T c5_y_data[], int32_T c5_y_size[2])
{
  emlrtMsgIdentifier c5_thisId;
  c5_thisId.fIdentifier = const_cast<const char_T *>(c5_identifier);
  c5_thisId.fParent = NULL;
  c5_thisId.bParentIsCell = false;
  c5_d_emlrt_marshallIn(chartInstance, sf_mex_dup(c5_scores), &c5_thisId,
                        c5_y_data, c5_y_size);
  sf_mex_destroy(&c5_scores);
}

static void c5_d_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId,
  real32_T c5_y_data[], int32_T c5_y_size[2])
{
  static boolean_T c5_b_bv[2] = { true, false };

  int32_T c5_fv_size[2];
  int32_T c5_b_i;
  int32_T c5_i;
  int32_T c5_i1;
  real32_T c5_fv_data[20];
  uint32_T c5_uv[2];
  boolean_T c5_bv[2];
  for (c5_i = 0; c5_i < 2; c5_i++) {
    c5_uv[c5_i] = (uint32_T)(-19 * c5_i) + 20U;
  }

  c5_fv_size[0] = sf_mex_get_dimension(c5_u, 0);
  c5_fv_size[1] = sf_mex_get_dimension(c5_u, 1);
  for (c5_b_i = 0; c5_b_i < 2; c5_b_i++) {
    c5_bv[c5_b_i] = c5_b_bv[c5_b_i];
  }

  sf_mex_import_vs(c5_parentId, sf_mex_dup(c5_u), &c5_fv_data, 0, 1, 0U, 1, 0U,
                   2, c5_bv, c5_uv, c5_fv_size);
  c5_y_size[0] = c5_fv_size[0];
  c5_y_size[1] = 1;
  for (c5_i1 = 0; c5_i1 < c5_fv_size[0] * c5_fv_size[1]; c5_i1++) {
    c5_y_data[c5_i1] = c5_fv_data[c5_i1];
  }

  sf_mex_destroy(&c5_u);
}

static uint8_T c5_e_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_b_is_active_c5_LaneDetection, const char_T
  *c5_identifier)
{
  emlrtMsgIdentifier c5_thisId;
  uint8_T c5_y;
  c5_thisId.fIdentifier = const_cast<const char_T *>(c5_identifier);
  c5_thisId.fParent = NULL;
  c5_thisId.bParentIsCell = false;
  c5_y = c5_f_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c5_b_is_active_c5_LaneDetection), &c5_thisId);
  sf_mex_destroy(&c5_b_is_active_c5_LaneDetection);
  return c5_y;
}

static uint8_T c5_f_emlrt_marshallIn(SFc5_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c5_u, const emlrtMsgIdentifier *c5_parentId)
{
  uint8_T c5_b_u;
  uint8_T c5_y;
  sf_mex_import(c5_parentId, sf_mex_dup(c5_u), &c5_b_u, 1, 3, 0U, 0, 0U, 0);
  c5_y = c5_b_u;
  sf_mex_destroy(&c5_u);
  return c5_y;
}

static void c5_emxConvertDynamicMatrixFromEmx_(SFc5_LaneDetectionInstanceStruct *
  chartInstance, c5_emxArray_real_T_20x4 *c5_emx, real_T c5_data[80], int32_T
  c5_size[2])
{
  memcpy((void *)c5_data, &c5_emx->data, sizeof(real_T) * (uint32_T)
         (c5_emx->size[0] * c5_emx->size[1]));
  memcpy((void *)c5_size, &c5_emx->size, sizeof(int32_T) << 1);
}

static void c5_b_emxConvertDynamicMatrixFromEmx_
  (SFc5_LaneDetectionInstanceStruct *chartInstance, c5_emxArray_real32_T_20x1
   *c5_emx, real32_T c5_data[20], int32_T c5_size[2])
{
  memcpy((void *)c5_data, &c5_emx->data, sizeof(real32_T) * (uint32_T)
         (c5_emx->size[0] * c5_emx->size[1]));
  memcpy((void *)c5_size, &c5_emx->size, sizeof(int32_T) << 1);
}

static __global__ __launch_bounds__(1024, 1) void c5_coder_reduce0(const
  real32_T c5_inputVar[150528], real32_T *c5_outputVar)
{
  real32_T c5_tmpRed0;
  real32_T c5_tmpRed1;
  uint32_T c5_blockStride;
  uint32_T c5_idx;
  uint32_T c5_m;
  uint32_T c5_mask;
  uint32_T c5_numActiveThreads;
  uint32_T c5_numActiveWarps;
  uint32_T c5_thBlkId;
  uint32_T c5_threadId;
  uint32_T c5_threadStride;
  c5_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c5_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c5_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c5_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c5_tmpRed0 = 0.0F;
  c5_tmpRed1 = 0.0F;
  c5_numActiveThreads = c5_blockStride;
  if (mwIsLastBlock()) {
    c5_m = 150527U % c5_blockStride;
    if (c5_m > 0U) {
      c5_numActiveThreads = c5_m;
    }
  }

  c5_numActiveWarps = ((c5_numActiveThreads + warpSize) - 1U) / warpSize;
  if (c5_threadId <= 150526U) {
    c5_tmpRed0 = c5_inputVar[c5_threadId];
    c5_tmpRed1 = c5_tmpRed0;
  }

  c5_mask = __ballot_sync(MAX_uint32_T, c5_threadId <= 150526U);
  for (c5_idx = c5_threadId + c5_threadStride; c5_idx <= 150526U; c5_idx +=
       c5_threadStride) {
    c5_tmpRed0 = c5_b_callFcn_device(c5_tmpRed0, c5_inputVar[c5_idx]);
    c5_tmpRed1 = c5_callFcn_device(c5_tmpRed1, c5_inputVar[c5_idx]);
  }

  c5_tmpRed0 = c5_workGroupReduction(c5_tmpRed0, c5_mask, c5_numActiveWarps);
  c5_tmpRed1 = c5_b_workGroupReduction(c5_tmpRed1, c5_mask, c5_numActiveWarps);
  if (c5_thBlkId == 0U) {
    c5_atomicOpreal32_T(&c5_outputVar[0], c5_tmpRed0);
    c5_b_atomicOpreal32_T(&c5_outputVar[1], c5_tmpRed1);
  }
}

static __device__ real32_T c5_threadGroupReduction(real32_T c5_val, uint32_T
  c5_lane, uint32_T c5_mask)
{
  real32_T c5_other;
  uint32_T c5_activeSize;
  uint32_T c5_offset;
  c5_activeSize = __popc(c5_mask);
  c5_offset = (c5_activeSize + 1U) / 2U;
  while (c5_activeSize > 1U) {
    c5_other = c5_shflDown1(c5_val, c5_offset, c5_mask);
    if (c5_lane + c5_offset < c5_activeSize) {
      c5_val = c5_b_callFcn_device(c5_val, c5_other);
    }

    c5_activeSize = c5_offset;
    c5_offset = (c5_offset + 1U) / 2U;
  }

  return c5_val;
}

static __device__ real32_T c5_shflDown1(real32_T c5_in1, uint32_T c5_offset,
  uint32_T c5_mask)
{
  int32_T *c5_tmp;
  c5_tmp = (int32_T *)&c5_in1;
  *c5_tmp = __shfl_down_sync(c5_mask, *c5_tmp, c5_offset);
  return *(real32_T *)c5_tmp;
}

static __device__ real32_T c5_workGroupReduction(real32_T c5_val, uint32_T
  c5_mask, uint32_T c5_numActiveWarps)
{
  __shared__ real32_T c5_shared[32];
  uint32_T c5_lane;
  uint32_T c5_thBlkId;
  uint32_T c5_widx;
  c5_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c5_lane = c5_thBlkId % warpSize;
  c5_widx = c5_thBlkId / warpSize;
  c5_val = c5_threadGroupReduction(c5_val, c5_lane, c5_mask);
  if (c5_lane == 0U) {
    c5_shared[c5_widx] = c5_val;
  }

  __syncthreads();
  c5_mask = __ballot_sync(MAX_uint32_T, c5_lane < c5_numActiveWarps);
  c5_val = c5_shared[c5_lane];
  if (c5_widx == 0U) {
    c5_val = c5_threadGroupReduction(c5_val, c5_lane, c5_mask);
  }

  return c5_val;
}

static __device__ real32_T c5_b_threadGroupReduction(real32_T c5_val, uint32_T
  c5_lane, uint32_T c5_mask)
{
  real32_T c5_other;
  uint32_T c5_activeSize;
  uint32_T c5_offset;
  c5_activeSize = __popc(c5_mask);
  c5_offset = (c5_activeSize + 1U) / 2U;
  while (c5_activeSize > 1U) {
    c5_other = c5_shflDown1(c5_val, c5_offset, c5_mask);
    if (c5_lane + c5_offset < c5_activeSize) {
      c5_val = c5_callFcn_device(c5_val, c5_other);
    }

    c5_activeSize = c5_offset;
    c5_offset = (c5_offset + 1U) / 2U;
  }

  return c5_val;
}

static __device__ real32_T c5_b_workGroupReduction(real32_T c5_val, uint32_T
  c5_mask, uint32_T c5_numActiveWarps)
{
  __shared__ real32_T c5_shared[32];
  uint32_T c5_lane;
  uint32_T c5_thBlkId;
  uint32_T c5_widx;
  c5_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c5_lane = c5_thBlkId % warpSize;
  c5_widx = c5_thBlkId / warpSize;
  c5_val = c5_b_threadGroupReduction(c5_val, c5_lane, c5_mask);
  if (c5_lane == 0U) {
    c5_shared[c5_widx] = c5_val;
  }

  __syncthreads();
  c5_mask = __ballot_sync(MAX_uint32_T, c5_lane < c5_numActiveWarps);
  c5_val = c5_shared[c5_lane];
  if (c5_widx == 0U) {
    c5_val = c5_b_threadGroupReduction(c5_val, c5_lane, c5_mask);
  }

  return c5_val;
}

static __device__ real32_T c5_atomicOpreal32_T(real32_T *c5_address, real32_T
  c5_value)
{
  uint32_T c5_assumed;
  uint32_T c5_old;
  uint32_T *c5_address_as_up;
  c5_address_as_up = (uint32_T *)c5_address;
  c5_old = *c5_address_as_up;
  do {
    c5_assumed = c5_old;
    c5_old = atomicCAS(c5_address_as_up, c5_old, __float_as_uint
                       (c5_b_callFcn_device(c5_value, __uint_as_float(c5_old))));
  } while (c5_assumed != c5_old);

  return __uint_as_float(c5_old);
}

static __device__ real32_T c5_b_atomicOpreal32_T(real32_T *c5_address, real32_T
  c5_value)
{
  uint32_T c5_assumed;
  uint32_T c5_old;
  uint32_T *c5_address_as_up;
  c5_address_as_up = (uint32_T *)c5_address;
  c5_old = *c5_address_as_up;
  do {
    c5_assumed = c5_old;
    c5_old = atomicCAS(c5_address_as_up, c5_old, __float_as_uint
                       (c5_callFcn_device(c5_value, __uint_as_float(c5_old))));
  } while (c5_assumed != c5_old);

  return __uint_as_float(c5_old);
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel1
  (int16_T c5_aux1[960])
{
  int32_T c5_i;
  c5_i = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i < 960) {
    if (c5_i + 1 <= 480) {
      c5_aux1[c5_i] = (int16_T)(c5_i + 1);
    } else {
      c5_aux1[c5_i] = (int16_T)(960 - c5_i);
    }
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel2
  (int16_T c5_aux2[1280])
{
  int32_T c5_i;
  c5_i = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i < 1280) {
    if (c5_i + 1 <= 640) {
      c5_aux2[c5_i] = (int16_T)(c5_i + 1);
    } else {
      c5_aux2[c5_i] = (int16_T)(1280 - c5_i);
    }
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel3(const
  int16_T c5_aux1[960], real_T c5_rowWeights[2016], int16_T c5_ipRowIndices[2016])
{
  real_T c5_absx;
  real_T c5_absx2;
  real_T c5_absx3;
  real_T c5_ipRowIdx;
  uint64_T c5_threadId;
  int32_T c5_k;
  int32_T c5_l;
  int32_T c5_oldIdx;
  int32_T c5_rowIdx;
  int32_T c5_rowIndices;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_k = (int32_T)(c5_threadId % 9UL);
  c5_rowIdx = (int32_T)((c5_threadId - (uint64_T)c5_k) / 9UL);
  if ((c5_rowIdx < 224) && (c5_k < 9)) {
    c5_ipRowIdx = ((real_T)c5_rowIdx + 1.0) / 0.46666666666666667 +
      -0.5714285714285714;
    c5_rowIndices = (int32_T)floor(c5_ipRowIdx - 4.2857142857142856);
    c5_absx = fabs(0.46666666666666667 * (c5_ipRowIdx - ((real_T)(c5_rowIndices
      + c5_k) + 1.0)));
    c5_absx2 = c5_absx * c5_absx;
    c5_absx3 = pow(c5_absx, 3.0);
    c5_oldIdx = (c5_rowIndices + c5_k) + 1;
    if (c5_oldIdx - 1 == 0) {
      c5_l = 0;
    } else {
      c5_l = (int32_T)fmod((real_T)c5_oldIdx - 1.0, 960.0);
      if ((c5_l != 0) && (c5_oldIdx - 1 < 0)) {
        c5_l += 960;
      }
    }

    c5_ipRowIndices[c5_rowIdx + 224 * c5_k] = c5_aux1[c5_l];
    c5_rowWeights[c5_rowIdx + 224 * c5_k] = 0.46666666666666667 * (((1.5 *
      c5_absx3 - 2.5 * c5_absx2) + 1.0) * (real_T)(c5_absx <= 1.0) + (((-0.5 *
      c5_absx3 + 2.5 * c5_absx2) - 4.0 * c5_absx) + 2.0) * (real_T)((1.0 <
      c5_absx) && (c5_absx <= 2.0)));
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel4(const
  int16_T c5_aux2[1280], real_T c5_colWeights[2688], int16_T c5_ipColIndices
  [2688])
{
  real_T c5_absx;
  real_T c5_absx2;
  real_T c5_absx3;
  real_T c5_ipColIdx;
  uint64_T c5_threadId;
  int32_T c5_colIdx;
  int32_T c5_colIndices;
  int32_T c5_k;
  int32_T c5_l;
  int32_T c5_oldIdx;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_k = (int32_T)(c5_threadId % 12UL);
  c5_colIdx = (int32_T)((c5_threadId - (uint64_T)c5_k) / 12UL);
  if ((c5_colIdx < 224) && (c5_k < 12)) {
    c5_ipColIdx = ((real_T)c5_colIdx + 1.0) / 0.35 + -0.9285714285714286;
    c5_colIndices = (int32_T)floor(c5_ipColIdx - 5.7142857142857144);
    c5_absx = fabs(0.35 * (c5_ipColIdx - ((real_T)(c5_colIndices + c5_k) + 1.0)));
    c5_absx2 = c5_absx * c5_absx;
    c5_absx3 = pow(c5_absx, 3.0);
    c5_oldIdx = (c5_colIndices + c5_k) + 1;
    if (c5_oldIdx - 1 == 0) {
      c5_l = 0;
    } else {
      c5_l = (int32_T)fmod((real_T)c5_oldIdx - 1.0, 1280.0);
      if ((c5_l != 0) && (c5_oldIdx - 1 < 0)) {
        c5_l += 1280;
      }
    }

    c5_ipColIndices[c5_colIdx + 224 * c5_k] = c5_aux2[c5_l];
    c5_colWeights[c5_colIdx + 224 * c5_k] = 0.35 * (((1.5 * c5_absx3 - 2.5 *
      c5_absx2) + 1.0) * (real_T)(c5_absx <= 1.0) + (((-0.5 * c5_absx3 + 2.5 *
      c5_absx2) - 4.0 * c5_absx) + 2.0) * (real_T)((1.0 < c5_absx) && (c5_absx <=
      2.0)));
  }
}

static __global__ __launch_bounds__(224, 1) void c5_eML_blk_kernel_kernel5(const
  real_T c5_rowWeights[2016], real_T c5_rowWeightsTotal[224])
{
  int32_T c5_j;
  c5_j = (int32_T)mwGetGlobalThreadIndex();
  if (c5_j < 224) {
    c5_rowWeightsTotal[c5_j] = c5_rowWeights[c5_j];
  }
}

static __global__ __launch_bounds__(224, 1) void c5_eML_blk_kernel_kernel6(const
  real_T c5_rowWeights[2016], const int32_T c5_xoffset, real_T
  c5_rowWeightsTotal[224])
{
  int32_T c5_j;
  c5_j = (int32_T)mwGetGlobalThreadIndex();
  if (c5_j < 224) {
    c5_rowWeightsTotal[c5_j] += c5_rowWeights[c5_xoffset + c5_j];
  }
}

static __global__ __launch_bounds__(224, 1) void c5_eML_blk_kernel_kernel7(const
  real_T c5_colWeights[2688], real_T c5_colWeightsTotal[224])
{
  int32_T c5_j;
  c5_j = (int32_T)mwGetGlobalThreadIndex();
  if (c5_j < 224) {
    c5_colWeightsTotal[c5_j] = c5_colWeights[c5_j];
  }
}

static __global__ __launch_bounds__(224, 1) void c5_eML_blk_kernel_kernel8(const
  real_T c5_colWeights[2688], const int32_T c5_xoffset, real_T
  c5_colWeightsTotal[224])
{
  int32_T c5_j;
  c5_j = (int32_T)mwGetGlobalThreadIndex();
  if (c5_j < 224) {
    c5_colWeightsTotal[c5_j] += c5_colWeights[c5_xoffset + c5_j];
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel9(const
  real_T c5_colWeightsTotal[224], const real_T c5_colWeights[2688], const
  real32_T c5_b_In[921600], const int16_T c5_ipColIndices[2688], real32_T
  c5_partialResize[322560])
{
  real_T c5_sumVal;
  uint64_T c5_threadId;
  uint64_T c5_tmpIndex;
  int32_T c5_colIdx;
  int32_T c5_dimIdx;
  int32_T c5_l;
  int32_T c5_rowIdx;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_dimIdx = (int32_T)(c5_threadId % 3UL);
  c5_tmpIndex = (c5_threadId - (uint64_T)c5_dimIdx) / 3UL;
  c5_colIdx = (int32_T)(c5_tmpIndex % 224UL);
  c5_tmpIndex = (c5_tmpIndex - (uint64_T)c5_colIdx) / 224UL;
  c5_rowIdx = (int32_T)c5_tmpIndex;
  if ((c5_rowIdx < 480) && (c5_colIdx < 224) && (c5_dimIdx < 3)) {
    c5_sumVal = 0.0;
    for (c5_l = 0; c5_l < 12; c5_l++) {
      c5_sumVal += (real_T)c5_b_In[(c5_rowIdx + 480 * ((int32_T)
        c5_ipColIndices[c5_colIdx + 224 * c5_l] - 1)) + 307200 * c5_dimIdx] *
        (c5_colWeights[c5_colIdx + 224 * c5_l] / c5_colWeightsTotal[c5_colIdx]);
    }

    c5_partialResize[(c5_rowIdx + 480 * c5_colIdx) + 107520 * c5_dimIdx] =
      (real32_T)c5_sumVal;
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel10(
  const real_T c5_rowWeightsTotal[224], const real_T c5_rowWeights[2016], const
  real32_T c5_partialResize[322560], const int16_T c5_ipRowIndices[2016],
  real32_T c5_b_out[150528])
{
  real_T c5_sumVal;
  uint64_T c5_threadId;
  uint64_T c5_tmpIndex;
  int32_T c5_colIdx;
  int32_T c5_dimIdx;
  int32_T c5_l;
  int32_T c5_rowIdx;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_dimIdx = (int32_T)(c5_threadId % 3UL);
  c5_tmpIndex = (c5_threadId - (uint64_T)c5_dimIdx) / 3UL;
  c5_rowIdx = (int32_T)(c5_tmpIndex % 224UL);
  c5_tmpIndex = (c5_tmpIndex - (uint64_T)c5_rowIdx) / 224UL;
  c5_colIdx = (int32_T)c5_tmpIndex;
  if ((c5_colIdx < 224) && (c5_rowIdx < 224) && (c5_dimIdx < 3)) {
    c5_sumVal = 0.0;
    for (c5_l = 0; c5_l < 9; c5_l++) {
      c5_sumVal += (real_T)c5_partialResize[(((int32_T)c5_ipRowIndices[c5_rowIdx
        + 224 * c5_l] + 480 * c5_colIdx) + 107520 * c5_dimIdx) - 1] *
        (c5_rowWeights[c5_rowIdx + 224 * c5_l] / c5_rowWeightsTotal[c5_rowIdx]);
    }

    c5_b_out[(c5_rowIdx + 224 * c5_colIdx) + 50176 * c5_dimIdx] = (real32_T)
      c5_sumVal;
  }
}

static __global__ __launch_bounds__(32, 1) void c5_eML_blk_kernel_kernel11
  (real32_T c5_b_out[150528], real32_T c5_outVal[2])
{
  int32_T c5_indV;
  c5_indV = (int32_T)mwGetGlobalThreadIndex();
  if (c5_indV < 2) {
    c5_outVal[c5_indV] = c5_b_out[150527];
    c5_outVal[c5_indV] = c5_b_out[150527];
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel12(
  const real32_T c5_y, const real32_T c5_outVal, real32_T c5_b_out[150528],
  real32_T c5_c_out[150528])
{
  int32_T c5_i2;
  c5_i2 = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i2 < 150528) {
    c5_c_out[c5_i2] = (c5_b_out[c5_i2] - c5_outVal) / c5_y;
  }
}

static __global__ __launch_bounds__(32, 1) void c5_eML_blk_kernel_kernel13(const
  int8_T c5_dv[8], real_T c5_anchors[8])
{
  int32_T c5_i3;
  c5_i3 = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i3 < 8) {
    c5_anchors[c5_i3] = (real_T)c5_dv[c5_i3];
  }
}

static __global__ __launch_bounds__(32, 1) void c5_eML_blk_kernel_kernel14(const
  real_T c5_dv1[4], real_T c5_anchors[4], real_T c5_b_anchors[8])
{
  int32_T c5_i4;
  c5_i4 = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i4 < 4) {
    c5_b_anchors[c5_i4] = c5_dv1[c5_i4];
    c5_anchors[c5_i4] = c5_b_anchors[c5_i4 + 4] / 16.0;
    c5_b_anchors[c5_i4 + 4] = c5_anchors[c5_i4];
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel15(
  const real_T c5_anchors[8], const real32_T c5_tmpFeatureMap[4704], real32_T
  c5_boxOut[4704])
{
  uint64_T c5_threadId;
  uint64_T c5_tmpIndex;
  int32_T c5_anchorIdx;
  int32_T c5_colIdx;
  int32_T c5_ind;
  int32_T c5_rowIdx;
  real32_T c5_bh;
  real32_T c5_bw;
  real32_T c5_cx;
  real32_T c5_cy;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_rowIdx = (int32_T)(c5_threadId % 14UL);
  c5_tmpIndex = (c5_threadId - (uint64_T)c5_rowIdx) / 14UL;
  c5_colIdx = (int32_T)(c5_tmpIndex % 14UL);
  c5_tmpIndex = (c5_tmpIndex - (uint64_T)c5_colIdx) / 14UL;
  c5_anchorIdx = (int32_T)c5_tmpIndex;
  if ((c5_anchorIdx < 4) && (c5_colIdx < 14) && (c5_rowIdx < 14)) {
    c5_ind = ((((c5_rowIdx * 14) << 2) + (c5_colIdx << 2)) + c5_anchorIdx) + 1;
    c5_cx = (c5_tmpFeatureMap[((c5_rowIdx + 14 * c5_colIdx) + 196 * c5_anchorIdx)
             + 784] + (real32_T)c5_colIdx) * 16.0F;
    c5_cy = (c5_tmpFeatureMap[((c5_rowIdx + 14 * c5_colIdx) + 196 * c5_anchorIdx)
             + 1568] + (real32_T)c5_rowIdx) * 16.0F;
    c5_bw = c5_tmpFeatureMap[((c5_rowIdx + 14 * c5_colIdx) + 196 * c5_anchorIdx)
      + 2352] * (real32_T)c5_anchors[c5_anchorIdx + 4] * 16.0F;
    c5_bh = c5_tmpFeatureMap[((c5_rowIdx + 14 * c5_colIdx) + 196 * c5_anchorIdx)
      + 3136] * (real32_T)c5_anchors[c5_anchorIdx] * 16.0F;
    c5_boxOut[c5_ind - 1] = c5_cx - c5_bw / 2.0F;
    c5_boxOut[c5_ind + 783] = c5_cy - c5_bh / 2.0F;
    c5_boxOut[c5_ind + 1567] = c5_cx + c5_bw / 2.0F;
    c5_boxOut[c5_ind + 2351] = c5_cy + c5_bh / 2.0F;
    c5_boxOut[c5_ind + 3135] = c5_tmpFeatureMap[(c5_rowIdx + 14 * c5_colIdx) +
      196 * c5_anchorIdx] * c5_tmpFeatureMap[((c5_rowIdx + 14 * c5_colIdx) + 196
      * c5_anchorIdx) + 3920];
    c5_boxOut[c5_ind + 3919] = 1.0F;
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel16(
  const real32_T c5_boxOut[4704], boolean_T c5_bv[784])
{
  int32_T c5_i5;
  c5_i5 = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i5 < 784) {
    c5_bv[c5_i5] = (boolean_T)((real_T)c5_boxOut[c5_i5 + 3136] >= 0.2);
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel17(
  const real32_T c5_boxOut[4704], const int16_T c5_ii_data[784], const int32_T
  c5_thresholdedPrediction_size[2], const int32_T c5_ii_size[1], real32_T
  c5_thresholdedPrediction_data[4704])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  uint64_T c5_tmpIndex;
  int32_T c5_i6;
  int32_T c5_i9;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = ((int64_T)(c5_ii_size[0] - 1) + 1L) * 6L - 1L;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i9 = (int32_T)(c5_idx % ((uint64_T)(c5_ii_size[0] - 1) + 1UL));
    c5_tmpIndex = (c5_idx - (uint64_T)c5_i9) / ((uint64_T)(c5_ii_size[0] - 1) +
      1UL);
    c5_i6 = (int32_T)c5_tmpIndex;
    c5_thresholdedPrediction_data[c5_i9 + c5_thresholdedPrediction_size[0] *
      c5_i6] = c5_boxOut[((int32_T)c5_ii_data[c5_i9] + 784 * c5_i6) - 1];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel18(
  const real32_T c5_thresholdedPrediction_data[4704], const int32_T
  c5_thresholdedPrediction_size[2], const int32_T c5_bboxesX1Y1X2Y2_size[2],
  const int32_T c5_i7, real_T c5_bboxesX1Y1X2Y2_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i13;
  int32_T c5_i8;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = ((int64_T)c5_i7 + 1L) * 4L - 1L;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i13 = (int32_T)(c5_idx % ((uint64_T)c5_i7 + 1UL));
    c5_i8 = (int32_T)((c5_idx - (uint64_T)c5_i13) / ((uint64_T)c5_i7 + 1UL));
    c5_bboxesX1Y1X2Y2_data[c5_i13 + c5_bboxesX1Y1X2Y2_size[0] * c5_i8] = (real_T)
      c5_thresholdedPrediction_data[c5_i13 + c5_thresholdedPrediction_size[0] *
      c5_i8];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel19(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_i10, real_T
  c5_x1_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i12;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i10;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i12 = (int32_T)c5_idx;
    c5_x1_data[c5_i12] = c5_bboxesX1Y1X2Y2_data[c5_i12];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel20(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_i14, real_T c5_y1_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i16;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i14;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i16 = (int32_T)c5_idx;
    c5_y1_data[c5_i16] = c5_bboxesX1Y1X2Y2_data[c5_i16 + c5_bboxesX1Y1X2Y2_size
      [0]];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel21(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_i17, real_T c5_x2_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i18;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i17;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i18 = (int32_T)c5_idx;
    c5_x2_data[c5_i18] = c5_bboxesX1Y1X2Y2_data[c5_i18 +
      (c5_bboxesX1Y1X2Y2_size[0] << 1)];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel22(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_i19, real_T c5_y2_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i20;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i19;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i20 = (int32_T)c5_idx;
    c5_y2_data[c5_i20] = c5_bboxesX1Y1X2Y2_data[c5_i20 + c5_bboxesX1Y1X2Y2_size
      [0] * 3];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel23(
  const int32_T c5_end, real_T c5_x1_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_end - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i = (int32_T)c5_idx;
    if (c5_x1_data[c5_i] < 1.0) {
      c5_x1_data[c5_i] = 1.0;
    }
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel24(
  const int32_T c5_end, real_T c5_y1_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_end - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i = (int32_T)c5_idx;
    if (c5_y1_data[c5_i] < 1.0) {
      c5_y1_data[c5_i] = 1.0;
    }
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel25(
  const int32_T c5_end, real_T c5_x2_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_end - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i = (int32_T)c5_idx;
    if (c5_x2_data[c5_i] > 224.0) {
      c5_x2_data[c5_i] = 224.0;
    }
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel26(
  const int32_T c5_end, real_T c5_y2_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_end - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i = (int32_T)c5_idx;
    if (c5_y2_data[c5_i] > 224.0) {
      c5_y2_data[c5_i] = 224.0;
    }
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel27(
  const real_T c5_x1_data[784], const int32_T c5_loop_ub, real_T
  c5_bboxesX1Y1X2Y2_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i21;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_loop_ub;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i21 = (int32_T)c5_idx;
    c5_bboxesX1Y1X2Y2_data[c5_i21] = c5_x1_data[c5_i21];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel28(
  const real_T c5_y1_data[784], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_loop_ub, real_T c5_bboxesX1Y1X2Y2_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i22;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_loop_ub;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i22 = (int32_T)c5_idx;
    c5_bboxesX1Y1X2Y2_data[c5_i22 + c5_bboxesX1Y1X2Y2_size[0]] =
      c5_y1_data[c5_i22];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel29(
  const real_T c5_x2_data[784], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_loop_ub, real_T c5_bboxesX1Y1X2Y2_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i23;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_loop_ub;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i23 = (int32_T)c5_idx;
    c5_bboxesX1Y1X2Y2_data[c5_i23 + (c5_bboxesX1Y1X2Y2_size[0] << 1)] =
      c5_x2_data[c5_i23];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel30(
  const real_T c5_y2_data[784], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_loop_ub, real_T c5_bboxesX1Y1X2Y2_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i24;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_loop_ub;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i24 = (int32_T)c5_idx;
    c5_bboxesX1Y1X2Y2_data[c5_i24 + c5_bboxesX1Y1X2Y2_size[0] * 3] =
      c5_y2_data[c5_i24];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel31(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T c5_loop_ub, real_T
  c5_bboxPred_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i28;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_loop_ub;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i28 = (int32_T)c5_idx;
    c5_bboxPred_data[c5_i28] = ((c5_bboxesX1Y1X2Y2_data[c5_i28] - 0.5) *
      2.8571428571428572 + -0.9285714285714286) + 0.5;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel32(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_bboxPred_size[2], const int32_T
  c5_loop_ub, real_T c5_bboxPred_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i29;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_loop_ub;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i29 = (int32_T)c5_idx;
    c5_bboxPred_data[c5_i29 + c5_bboxPred_size[0]] =
      ((c5_bboxesX1Y1X2Y2_data[c5_i29 + c5_bboxesX1Y1X2Y2_size[0]] - 0.5) *
       2.1428571428571428 + -0.5714285714285714) + 0.5;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel33(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_bboxPred_size[2], const int32_T
  c5_loop_ub, real_T c5_bboxPred_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i30;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_loop_ub;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i30 = (int32_T)c5_idx;
    c5_bboxPred_data[c5_i30 + (c5_bboxPred_size[0] << 1)] =
      ((c5_bboxesX1Y1X2Y2_data[c5_i30 + (c5_bboxesX1Y1X2Y2_size[0] << 1)] + 0.5)
       * 2.8571428571428572 + -0.9285714285714286) - 0.5;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel34(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_bboxPred_size[2], const int32_T
  c5_loop_ub, real_T c5_bboxPred_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i31;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_loop_ub;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i31 = (int32_T)c5_idx;
    c5_bboxPred_data[c5_i31 + c5_bboxPred_size[0] * 3] =
      ((c5_bboxesX1Y1X2Y2_data[c5_i31 + c5_bboxesX1Y1X2Y2_size[0] * 3] + 0.5) *
       2.1428571428571428 + -0.5714285714285714) - 0.5;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel35(
  const int32_T c5_nx, real_T c5_bboxPred_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_k;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_nx - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_k = (int32_T)c5_idx;
    c5_bboxPred_data[c5_k] = floor(c5_bboxPred_data[c5_k]);
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel36(
  const real_T c5_bboxPred_data[3136], const int32_T c5_bboxPred_size[2], const
  int32_T c5_i32, real_T c5_b_bboxPred_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i33;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i32;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i33 = (int32_T)c5_idx;
    c5_b_bboxPred_data[c5_i33] = (c5_bboxPred_data[c5_i33 + (c5_bboxPred_size[0]
      << 1)] - c5_bboxPred_data[c5_i33]) + 1.0;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel37(
  const real_T c5_bboxPred_data[784], const int32_T c5_bboxPred_size[2], const
  int32_T c5_b_bboxPred_size[1], real_T c5_b_bboxPred_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i34;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_b_bboxPred_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i34 = (int32_T)c5_idx;
    c5_b_bboxPred_data[c5_i34 + (c5_bboxPred_size[0] << 1)] =
      c5_bboxPred_data[c5_i34];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel38(
  const real_T c5_bboxPred_data[3136], const int32_T c5_bboxPred_size[2], const
  int32_T c5_i35, real_T c5_b_bboxPred_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i36;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i35;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i36 = (int32_T)c5_idx;
    c5_b_bboxPred_data[c5_i36] = (c5_bboxPred_data[c5_i36 + c5_bboxPred_size[0] *
      3] - c5_bboxPred_data[c5_i36 + c5_bboxPred_size[0]]) + 1.0;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel39(
  const real_T c5_bboxPred_data[784], const int32_T c5_bboxPred_size[2], const
  int32_T c5_b_bboxPred_size[1], real_T c5_b_bboxPred_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i37;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_b_bboxPred_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i37 = (int32_T)c5_idx;
    c5_b_bboxPred_data[c5_i37 + c5_bboxPred_size[0] * 3] =
      c5_bboxPred_data[c5_i37];
  }
}

static __global__ __launch_bounds__(32, 1) void c5_eML_blk_kernel_kernel40(const
  real_T c5_bboxPred_data[3136], const int32_T c5_bboxPred_size[2], const
  int32_T c5_i, const int32_T c5_b_bboxPred_size[2], const int32_T c5_count,
  real_T c5_b_bboxPred_data[3136])
{
  int32_T c5_i41;
  c5_i41 = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i41 < 4) {
    c5_b_bboxPred_data[c5_count + c5_b_bboxPred_size[0] * c5_i41] =
      c5_bboxPred_data[c5_i + c5_bboxPred_size[0] * c5_i41];
  }
}

static __global__ __launch_bounds__(32, 1) void c5_eML_blk_kernel_kernel41(const
  real32_T c5_thresholdedPrediction_data[4704], const int32_T
  c5_thresholdedPrediction_size[2], const int32_T c5_i, const real_T c5_count,
  real32_T c5_classPred_data[784], real32_T c5_scorePred_data[784])
{
  int32_T c5_tmpIdx;
  c5_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c5_tmpIdx < 1) {
    c5_scorePred_data[(int32_T)c5_count - 1] =
      c5_thresholdedPrediction_data[c5_i + (c5_thresholdedPrediction_size[0] <<
      2)];
    c5_classPred_data[(int32_T)c5_count - 1] =
      c5_thresholdedPrediction_data[c5_i + c5_thresholdedPrediction_size[0] * 5];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel42(
  const int32_T c5_i38, const int32_T c5_i39, int32_T c5_idx_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i40;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_i39 - c5_i38);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i40 = (int32_T)c5_idx;
    c5_idx_data[c5_i40] = c5_i38 + c5_i40;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel43(
  const int32_T c5_bboxPred_size[2], boolean_T c5_b_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i42;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_bboxPred_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i42 = (int32_T)c5_idx;
    c5_b_data[c5_i42] = false;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel44(
  const int32_T c5_b_size[2], const boolean_T c5_b_data[784], int32_T c5_i3,
  int32_T *c5_n)
{
  int64_T c5_loopEnd;
  int32_T c5_tmpRed0;
  uint32_T c5_blockStride;
  uint32_T c5_idx;
  uint32_T c5_m;
  uint32_T c5_mask;
  uint32_T c5_numActiveThreads;
  uint32_T c5_numActiveWarps;
  uint32_T c5_thBlkId;
  uint32_T c5_threadId;
  uint32_T c5_threadStride;
  c5_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c5_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c5_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c5_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c5_tmpRed0 = 0;
  c5_loopEnd = (int64_T)(c5_i3 - 1);
  c5_numActiveThreads = c5_blockStride;
  if (mwIsLastBlock()) {
    c5_m = ((int64_T)(c5_i3 - 1) + 1L) % (int64_T)c5_blockStride;
    if (c5_m > 0U) {
      c5_numActiveThreads = c5_m;
    }
  }

  c5_numActiveWarps = (uint32_T)(c5_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c5_threadId <= c5_loopEnd) {
    c5_tmpRed0 = (int32_T)c5_b_data[c5_threadId];
  }

  c5_mask = __ballot_sync(MAX_uint32_T, (int64_T)c5_threadId <= c5_loopEnd);
  for (c5_idx = c5_threadId + c5_threadStride; c5_idx <= (uint32_T)c5_loopEnd;
       c5_idx += c5_threadStride) {
    c5_tmpRed0 += (int32_T)c5_b_data[c5_idx];
  }

  c5_tmpRed0 = c5_c_workGroupReduction(c5_tmpRed0, c5_mask, c5_numActiveWarps);
  if (c5_thBlkId == 0U) {
    atomicAdd(&c5_n[0], c5_tmpRed0);
  }
}

static __device__ int32_T c5_c_threadGroupReduction(int32_T c5_val, uint32_T
  c5_lane, uint32_T c5_mask)
{
  int32_T c5_other;
  uint32_T c5_activeSize;
  uint32_T c5_offset;
  c5_activeSize = __popc(c5_mask);
  c5_offset = (c5_activeSize + 1U) / 2U;
  while (c5_activeSize > 1U) {
    c5_other = c5_b_shflDown1(c5_val, c5_offset, c5_mask);
    if (c5_lane + c5_offset < c5_activeSize) {
      c5_val += c5_other;
    }

    c5_activeSize = c5_offset;
    c5_offset = (c5_offset + 1U) / 2U;
  }

  return c5_val;
}

static __device__ int32_T c5_b_shflDown1(int32_T c5_in1, uint32_T c5_offset,
  uint32_T c5_mask)
{
  c5_in1 = __shfl_down_sync(c5_mask, c5_in1, c5_offset);
  return c5_in1;
}

static __device__ int32_T c5_c_workGroupReduction(int32_T c5_val, uint32_T
  c5_mask, uint32_T c5_numActiveWarps)
{
  __shared__ int32_T c5_shared[32];
  uint32_T c5_lane;
  uint32_T c5_thBlkId;
  uint32_T c5_widx;
  c5_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c5_lane = c5_thBlkId % warpSize;
  c5_widx = c5_thBlkId / warpSize;
  c5_val = c5_c_threadGroupReduction(c5_val, c5_lane, c5_mask);
  if (c5_lane == 0U) {
    c5_shared[c5_widx] = c5_val;
  }

  __syncthreads();
  c5_mask = __ballot_sync(MAX_uint32_T, c5_lane < c5_numActiveWarps);
  c5_val = c5_shared[c5_lane];
  if (c5_widx == 0U) {
    c5_val = c5_c_threadGroupReduction(c5_val, c5_lane, c5_mask);
  }

  return c5_val;
}

static __global__ __launch_bounds__(32, 1) void c5_eML_blk_kernel_kernel45(const
  int32_T *c5_n, const int32_T c5_bboxPred_size[2], int32_T *c5_nrows)
{
  int32_T c5_tmpIdx;
  c5_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c5_tmpIdx < 1) {
    *c5_nrows = c5_bboxPred_size[0] - *c5_n;
  }
}

static __global__ __launch_bounds__(32, 1) void c5_eML_blk_kernel_kernel46(const
  int32_T c5_bboxPred_size[2], const int32_T *c5_nrows, const int32_T
  c5_idx_data[784], real_T c5_bboxPred_data[3136])
{
  int32_T c5_b_i;
  int32_T c5_i;
  int32_T c5_i2;
  int32_T c5_j;
  c5_j = (int32_T)mwGetGlobalThreadIndex();
  if (c5_j < 4) {
    c5_i2 = c5_idx_data[0];
    for (c5_i = 0; c5_i <= *c5_nrows - c5_i2; c5_i++) {
      c5_b_i = c5_i2 + c5_i;
      c5_bboxPred_data[(c5_b_i + c5_bboxPred_size[0] * c5_j) - 1] =
        c5_bboxPred_data[c5_b_i + c5_bboxPred_size[0] * c5_j];
    }
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel47(
  const real_T c5_bboxPred_data[3136], const int32_T c5_bboxPred_size[2], const
  int32_T c5_b_bboxPred_size[2], const int32_T c5_i4, real_T c5_b_bboxPred_data
  [3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i43;
  int32_T c5_i46;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = ((int64_T)c5_i4 + 1L) * 4L - 1L;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i46 = (int32_T)(c5_idx % ((uint64_T)c5_i4 + 1UL));
    c5_i43 = (int32_T)((c5_idx - (uint64_T)c5_i46) / ((uint64_T)c5_i4 + 1UL));
    c5_b_bboxPred_data[c5_i46 + c5_b_bboxPred_size[0] * c5_i43] =
      c5_bboxPred_data[c5_i46 + c5_bboxPred_size[0] * c5_i43];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel48(
  const real_T c5_bboxPred_data[3136], const int32_T c5_bboxPred_size[2], real_T
  c5_b_bboxPred_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i44;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_bboxPred_size[0] * 4 - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i44 = (int32_T)c5_idx;
    c5_b_bboxPred_data[c5_i44] = c5_bboxPred_data[c5_i44];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel49(
  const int32_T c5_i45, const int32_T c5_i47, int32_T c5_idx_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i48;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_i47 - c5_i45);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i48 = (int32_T)c5_idx;
    c5_idx_data[c5_i48] = c5_i45 + c5_i48;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel50(
  const int32_T c5_scorePred_size[1], boolean_T c5_b_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i49;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_scorePred_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i49 = (int32_T)c5_idx;
    c5_b_data[c5_i49] = false;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel51(
  const int32_T c5_b_size[2], const boolean_T c5_b_data[784], int32_T c5_i6,
  int32_T *c5_n)
{
  int64_T c5_loopEnd;
  int32_T c5_tmpRed0;
  uint32_T c5_blockStride;
  uint32_T c5_idx;
  uint32_T c5_m;
  uint32_T c5_mask;
  uint32_T c5_numActiveThreads;
  uint32_T c5_numActiveWarps;
  uint32_T c5_thBlkId;
  uint32_T c5_threadId;
  uint32_T c5_threadStride;
  c5_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c5_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c5_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c5_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c5_tmpRed0 = 0;
  c5_loopEnd = (int64_T)(c5_i6 - 1);
  c5_numActiveThreads = c5_blockStride;
  if (mwIsLastBlock()) {
    c5_m = ((int64_T)(c5_i6 - 1) + 1L) % (int64_T)c5_blockStride;
    if (c5_m > 0U) {
      c5_numActiveThreads = c5_m;
    }
  }

  c5_numActiveWarps = (uint32_T)(c5_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c5_threadId <= c5_loopEnd) {
    c5_tmpRed0 = (int32_T)c5_b_data[c5_threadId];
  }

  c5_mask = __ballot_sync(MAX_uint32_T, (int64_T)c5_threadId <= c5_loopEnd);
  for (c5_idx = c5_threadId + c5_threadStride; c5_idx <= (uint32_T)c5_loopEnd;
       c5_idx += c5_threadStride) {
    c5_tmpRed0 += (int32_T)c5_b_data[c5_idx];
  }

  c5_tmpRed0 = c5_c_workGroupReduction(c5_tmpRed0, c5_mask, c5_numActiveWarps);
  if (c5_thBlkId == 0U) {
    atomicAdd(&c5_n[0], c5_tmpRed0);
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel52(
  const int32_T c5_i50, const int32_T c5_i51, int32_T c5_idx_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i52;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_i51 - c5_i50);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i52 = (int32_T)c5_idx;
    c5_idx_data[c5_i52] = c5_i50 + c5_i52;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel53(
  const int32_T c5_classPred_size[1], boolean_T c5_b_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i53;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_classPred_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i53 = (int32_T)c5_idx;
    c5_b_data[c5_i53] = false;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel54(
  const int32_T c5_b_size[2], const boolean_T c5_b_data[784], int32_T c5_i9,
  int32_T *c5_n)
{
  int64_T c5_loopEnd;
  int32_T c5_tmpRed0;
  uint32_T c5_blockStride;
  uint32_T c5_idx;
  uint32_T c5_m;
  uint32_T c5_mask;
  uint32_T c5_numActiveThreads;
  uint32_T c5_numActiveWarps;
  uint32_T c5_thBlkId;
  uint32_T c5_threadId;
  uint32_T c5_threadStride;
  c5_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c5_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c5_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c5_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c5_tmpRed0 = 0;
  c5_loopEnd = (int64_T)(c5_i9 - 1);
  c5_numActiveThreads = c5_blockStride;
  if (mwIsLastBlock()) {
    c5_m = ((int64_T)(c5_i9 - 1) + 1L) % (int64_T)c5_blockStride;
    if (c5_m > 0U) {
      c5_numActiveThreads = c5_m;
    }
  }

  c5_numActiveWarps = (uint32_T)(c5_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c5_threadId <= c5_loopEnd) {
    c5_tmpRed0 = (int32_T)c5_b_data[c5_threadId];
  }

  c5_mask = __ballot_sync(MAX_uint32_T, (int64_T)c5_threadId <= c5_loopEnd);
  for (c5_idx = c5_threadId + c5_threadStride; c5_idx <= (uint32_T)c5_loopEnd;
       c5_idx += c5_threadStride) {
    c5_tmpRed0 += (int32_T)c5_b_data[c5_idx];
  }

  c5_tmpRed0 = c5_c_workGroupReduction(c5_tmpRed0, c5_mask, c5_numActiveWarps);
  if (c5_thBlkId == 0U) {
    atomicAdd(&c5_n[0], c5_tmpRed0);
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel55(
  const real32_T c5_scorePred_data[784], const int32_T c5_scorePred_size[1],
  real32_T c5_out_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i58;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_scorePred_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i58 = (int32_T)c5_idx;
    c5_out_data[c5_i58] = c5_scorePred_data[c5_i58];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel56(
  const uint32_T c5_dv2[2], real_T c5_x1_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i62;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)((int32_T)c5_dv2[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i62 = (int32_T)c5_idx;
    c5_x1_data[c5_i62] = 0.0;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel57(
  const real_T c5_bboxPred_data[3136], const int32_T c5_bboxPred_size[2], const
  real_T c5_x1_data[784], const int32_T c5_bboxesX1Y1X2Y2_size[2], const int32_T
  c5_x1_size[1], real_T c5_bboxesX1Y1X2Y2_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  uint64_T c5_tmpIndex;
  int32_T c5_i56;
  int32_T c5_i59;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = ((int64_T)(c5_x1_size[0] - 1) + 1L) * 4L - 1L;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i59 = (int32_T)(c5_idx % ((uint64_T)(c5_x1_size[0] - 1) + 1UL));
    c5_tmpIndex = (c5_idx - (uint64_T)c5_i59) / ((uint64_T)(c5_x1_size[0] - 1) +
      1UL);
    c5_i56 = (int32_T)c5_tmpIndex;
    c5_bboxesX1Y1X2Y2_data[c5_i59 + c5_bboxesX1Y1X2Y2_size[0] * c5_i56] =
      c5_bboxPred_data[((int32_T)c5_x1_data[c5_i59] + c5_bboxPred_size[0] *
                        c5_i56) - 1];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel58(
  const real32_T c5_classPred_data[784], const real_T c5_x1_data[784], const
  int32_T c5_x1_size[1], real_T c5_y1_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i57;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_x1_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i57 = (int32_T)c5_idx;
    c5_y1_data[c5_i57] = (real_T)c5_classPred_data[(int32_T)c5_x1_data[c5_i57] -
      1];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel59(
  const int32_T c5_x1_size[1], boolean_T c5_selectedIndex_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i60;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_x1_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i60 = (int32_T)c5_idx;
    c5_selectedIndex_data[c5_i60] = true;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel60(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_i61, real_T c5_area_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i63;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i61;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i63 = (int32_T)c5_idx;
    c5_area_data[c5_i63] = c5_bboxesX1Y1X2Y2_data[c5_i63 +
      (c5_bboxesX1Y1X2Y2_size[0] << 1)] * c5_bboxesX1Y1X2Y2_data[c5_i63 +
      c5_bboxesX1Y1X2Y2_size[0] * 3];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel61(
  const int32_T c5_bboxesX1Y1X2Y2_size[2], const real_T c5_bboxesX1Y1X2Y2_data
  [3136], const int32_T c5_i64, real_T c5_x2_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i65;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i64;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i65 = (int32_T)c5_idx;
    c5_x2_data[c5_i65] = c5_bboxesX1Y1X2Y2_data[c5_i65] +
      c5_bboxesX1Y1X2Y2_data[c5_i65 + (c5_bboxesX1Y1X2Y2_size[0] << 1)];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel62(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], const int32_T c5_i66, real_T c5_y2_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i67;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)c5_i66;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i67 = (int32_T)c5_idx;
    c5_y2_data[c5_i67] = c5_bboxesX1Y1X2Y2_data[c5_i67 + c5_bboxesX1Y1X2Y2_size
      [0]] + c5_bboxesX1Y1X2Y2_data[c5_i67 + c5_bboxesX1Y1X2Y2_size[0] * 3];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel63(
  const int32_T c5_i12, const int32_T c5_iv[2], boolean_T c5_selectedIndex_data
  [784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i68;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_iv[1] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i68 = (int32_T)c5_idx;
    c5_selectedIndex_data[c5_i12 + c5_i68] = false;
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel64(
  const boolean_T c5_selectedIndex_data[784], const real_T c5_x1_data[784],
  const int32_T c5_selectedIndex_size[1], boolean_T c5_index_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i69;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_selectedIndex_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i69 = (int32_T)c5_idx;
    c5_index_data[(int32_T)c5_x1_data[c5_i69] - 1] =
      c5_selectedIndex_data[c5_i69];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel65(
  const real_T c5_bboxPred_data[3136], const int32_T c5_bboxPred_size[2], const
  int16_T c5_iv1_data[784], const int32_T c5_bboxesX1Y1X2Y2_size[2], const
  int32_T c5_iv1_size[1], real_T c5_bboxesX1Y1X2Y2_data[3136])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  uint64_T c5_tmpIndex;
  int32_T c5_i70;
  int32_T c5_i71;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = ((int64_T)(c5_iv1_size[0] - 1) + 1L) * 4L - 1L;
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i71 = (int32_T)(c5_idx % ((uint64_T)(c5_iv1_size[0] - 1) + 1UL));
    c5_tmpIndex = (c5_idx - (uint64_T)c5_i71) / ((uint64_T)(c5_iv1_size[0] - 1)
      + 1UL);
    c5_i70 = (int32_T)c5_tmpIndex;
    c5_bboxesX1Y1X2Y2_data[c5_i71 + c5_bboxesX1Y1X2Y2_size[0] * c5_i70] =
      c5_bboxPred_data[((int32_T)c5_iv1_data[c5_i71] + c5_bboxPred_size[0] *
                        c5_i70) - 1];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel66(
  const real32_T c5_scorePred_data[784], const int32_T c5_scorePred_size[1],
  real32_T c5_b_scores_data[784])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i55;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_scorePred_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i55 = (int32_T)c5_idx;
    c5_b_scores_data[c5_i55] = c5_scorePred_data[c5_i55];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel67(
  const real_T c5_bboxesX1Y1X2Y2_data[3136], const int32_T
  c5_bboxesX1Y1X2Y2_size[2], real_T c5_b_bboxes_data[])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i11;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_bboxesX1Y1X2Y2_size[0] * 4 - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i11 = (int32_T)c5_idx;
    c5_b_bboxes_data[c5_i11] = c5_bboxesX1Y1X2Y2_data[c5_i11];
  }
}

static __global__ __launch_bounds__(1024, 1) void c5_eML_blk_kernel_kernel68(
  const real32_T c5_b_scores_data[784], const int32_T c5_scores_size[1],
  real32_T c5_c_scores_data[])
{
  int64_T c5_loopEnd;
  uint64_T c5_idx;
  uint64_T c5_threadId;
  uint64_T c5_threadStride;
  int32_T c5_i15;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_threadStride = mwGetTotalThreadsLaunched();
  c5_loopEnd = (int64_T)(c5_scores_size[0] - 1);
  for (c5_idx = c5_threadId; c5_idx <= (uint64_T)c5_loopEnd; c5_idx +=
       c5_threadStride) {
    c5_i15 = (int32_T)c5_idx;
    c5_c_scores_data[c5_i15] = c5_b_scores_data[c5_i15];
  }
}

static __global__ __launch_bounds__(512, 1) void
  c5_DeepLearningNetwork_activations_kernel69(const real32_T c5_varargin_1
  [150528], c5_cell_wrap_18 *c5_r)
{
  int32_T c5_i;
  c5_i = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i < 150528) {
    c5_r->f1[c5_i] = c5_varargin_1[c5_i];
  }
}

static __global__ __launch_bounds__(512, 1) void
  c5_DeepLearningNetwork_activations_kernel70(const c5_cell_wrap_18 *c5_r,
  c5_cell_wrap_18 c5_miniBatchT[1])
{
  uint64_T c5_threadId;
  uint64_T c5_tmpIndex;
  int32_T c5_i1;
  int32_T c5_i2;
  int32_T c5_p;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_i1 = (int32_T)(c5_threadId % 224UL);
  c5_tmpIndex = (c5_threadId - (uint64_T)c5_i1) / 224UL;
  c5_i2 = (int32_T)(c5_tmpIndex % 224UL);
  c5_tmpIndex = (c5_tmpIndex - (uint64_T)c5_i2) / 224UL;
  c5_p = (int32_T)c5_tmpIndex;
  if ((c5_p < 3) && (c5_i2 < 224) && (c5_i1 < 224)) {
    c5_miniBatchT[0].f1[(c5_i1 + 224 * c5_i2) + 50176 * c5_p] = c5_r->f1[(c5_i2
      + 224 * c5_i1) + 50176 * c5_p];
  }
}

static __global__ __launch_bounds__(512, 1) void
  c5_DeepLearningNetwork_activations_kernel71(const real32_T c5_outMiniBatch
  [4704], real32_T c5_b_out[4704])
{
  uint64_T c5_threadId;
  uint64_T c5_tmpIndex;
  int32_T c5_i3;
  int32_T c5_i4;
  int32_T c5_p;
  c5_threadId = mwGetGlobalThreadIndex();
  c5_i3 = (int32_T)(c5_threadId % 14UL);
  c5_tmpIndex = (c5_threadId - (uint64_T)c5_i3) / 14UL;
  c5_i4 = (int32_T)(c5_tmpIndex % 14UL);
  c5_tmpIndex = (c5_tmpIndex - (uint64_T)c5_i4) / 14UL;
  c5_p = (int32_T)c5_tmpIndex;
  if ((c5_p < 24) && (c5_i4 < 14) && (c5_i3 < 14)) {
    c5_b_out[(c5_i3 + 14 * c5_i4) + 196 * c5_p] = c5_outMiniBatch[(c5_i4 + 14 *
      c5_i3) + 196 * c5_p];
  }
}

static __device__ real32_T c5_callFcn_device(real32_T c5_input1, real32_T
  c5_input2)
{
  return fmaxf(c5_input1, c5_input2);
}

static __device__ real32_T c5_b_callFcn_device(real32_T c5_input1, real32_T
  c5_input2)
{
  return fminf(c5_input1, c5_input2);
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::allocate()
{
  int32_T c5_idx;
  this->targetImpl->allocate(802816, 4);
  for (c5_idx = 0; c5_idx < 57; c5_idx++) {
    this->layers[c5_idx]->allocate();
  }

  (static_cast<MWTensor<real32_T> *>(this->inputTensors[0]))->setData
    (this->layers[0]->getLayerOutput(0));
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::postsetup()
{
  this->targetImpl->postSetup(this->layers, this->numLayers);
}

c5_yolov2ResNet50VehicleExample0_LaneDetection0::
  c5_yolov2ResNet50VehicleExample0_LaneDetection0()
{
  this->numLayers = 57;
  this->isInitialized = false;
  this->targetImpl = 0;
  this->layers[0] = new MWInputLayer;
  this->layers[0]->setName("input_1");
  this->layers[1] = new MWElementwiseAffineLayer;
  this->layers[1]->setName("input_1_normalization");
  this->layers[1]->setInPlaceIndex(0, 0);
  this->layers[2] = new MWFusedConvReLULayer;
  this->layers[2]->setName("conv1_activation_1_relu");
  this->layers[3] = new MWMaxPoolingLayer;
  this->layers[3]->setName("max_pooling2d_1");
  this->layers[4] = new MWFusedConvReLULayer;
  this->layers[4]->setName("res2a_branch2a_activation_2_relu");
  this->layers[5] = new MWFusedConvReLULayer;
  this->layers[5]->setName("res2a_branch2b_activation_3_relu");
  this->layers[6] = new MWConvLayer;
  this->layers[6]->setName("res2a_branch1");
  this->layers[7] = new MWFusedConvReLULayer;
  this->layers[7]->setName("res2a_branch2c_activation_4_relu");
  this->layers[7]->setInPlaceIndex(0, 1);
  this->layers[8] = new MWFusedConvReLULayer;
  this->layers[8]->setName("res2b_branch2a_activation_5_relu");
  this->layers[9] = new MWFusedConvReLULayer;
  this->layers[9]->setName("res2b_branch2b_activation_6_relu");
  this->layers[10] = new MWFusedConvReLULayer;
  this->layers[10]->setName("res2b_branch2c_activation_7_relu");
  this->layers[10]->setInPlaceIndex(0, 1);
  this->layers[11] = new MWFusedConvReLULayer;
  this->layers[11]->setName("res2c_branch2a_activation_8_relu");
  this->layers[12] = new MWFusedConvReLULayer;
  this->layers[12]->setName("res2c_branch2b_activation_9_relu");
  this->layers[13] = new MWMaxPoolingLayer;
  this->layers[13]->setName("downsample_add_3");
  this->layers[14] = new MWFusedConvReLULayer;
  this->layers[14]->setName("res2c_branch2c_activation_10_relu");
  this->layers[14]->setInPlaceIndex(0, 1);
  this->layers[15] = new MWFusedConvReLULayer;
  this->layers[15]->setName("res3a_branch2a_activation_11_relu");
  this->layers[16] = new MWFusedConvReLULayer;
  this->layers[16]->setName("res3a_branch2b_activation_12_relu");
  this->layers[17] = new MWConvLayer;
  this->layers[17]->setName("res3a_branch1");
  this->layers[18] = new MWFusedConvReLULayer;
  this->layers[18]->setName("res3a_branch2c_activation_13_relu");
  this->layers[18]->setInPlaceIndex(0, 1);
  this->layers[19] = new MWFusedConvReLULayer;
  this->layers[19]->setName("res3b_branch2a_activation_14_relu");
  this->layers[20] = new MWFusedConvReLULayer;
  this->layers[20]->setName("res3b_branch2b_activation_15_relu");
  this->layers[21] = new MWFusedConvReLULayer;
  this->layers[21]->setName("res3b_branch2c_activation_16_relu");
  this->layers[21]->setInPlaceIndex(0, 1);
  this->layers[22] = new MWFusedConvReLULayer;
  this->layers[22]->setName("res3c_branch2a_activation_17_relu");
  this->layers[23] = new MWFusedConvReLULayer;
  this->layers[23]->setName("res3c_branch2b_activation_18_relu");
  this->layers[24] = new MWFusedConvReLULayer;
  this->layers[24]->setName("res3c_branch2c_activation_19_relu");
  this->layers[24]->setInPlaceIndex(0, 1);
  this->layers[25] = new MWFusedConvReLULayer;
  this->layers[25]->setName("res3d_branch2a_activation_20_relu");
  this->layers[26] = new MWFusedConvReLULayer;
  this->layers[26]->setName("res3d_branch2b_activation_21_relu");
  this->layers[27] = new MWMaxPoolingLayer;
  this->layers[27]->setName("downsample_add_7");
  this->layers[28] = new MWFusedConvReLULayer;
  this->layers[28]->setName("res3d_branch2c_activation_22_relu");
  this->layers[28]->setInPlaceIndex(0, 1);
  this->layers[29] = new MWFusedConvReLULayer;
  this->layers[29]->setName("res4a_branch2a_activation_23_relu");
  this->layers[30] = new MWFusedConvReLULayer;
  this->layers[30]->setName("res4a_branch2b_activation_24_relu");
  this->layers[31] = new MWConvLayer;
  this->layers[31]->setName("res4a_branch1");
  this->layers[32] = new MWFusedConvReLULayer;
  this->layers[32]->setName("res4a_branch2c_activation_25_relu");
  this->layers[32]->setInPlaceIndex(0, 1);
  this->layers[33] = new MWFusedConvReLULayer;
  this->layers[33]->setName("res4b_branch2a_activation_26_relu");
  this->layers[34] = new MWFusedConvReLULayer;
  this->layers[34]->setName("res4b_branch2b_activation_27_relu");
  this->layers[35] = new MWFusedConvReLULayer;
  this->layers[35]->setName("res4b_branch2c_activation_28_relu");
  this->layers[35]->setInPlaceIndex(0, 1);
  this->layers[36] = new MWFusedConvReLULayer;
  this->layers[36]->setName("res4c_branch2a_activation_29_relu");
  this->layers[37] = new MWFusedConvReLULayer;
  this->layers[37]->setName("res4c_branch2b_activation_30_relu");
  this->layers[38] = new MWFusedConvReLULayer;
  this->layers[38]->setName("res4c_branch2c_activation_31_relu");
  this->layers[38]->setInPlaceIndex(0, 1);
  this->layers[39] = new MWFusedConvReLULayer;
  this->layers[39]->setName("res4d_branch2a_activation_32_relu");
  this->layers[40] = new MWFusedConvReLULayer;
  this->layers[40]->setName("res4d_branch2b_activation_33_relu");
  this->layers[41] = new MWFusedConvReLULayer;
  this->layers[41]->setName("res4d_branch2c_activation_34_relu");
  this->layers[41]->setInPlaceIndex(0, 1);
  this->layers[42] = new MWFusedConvReLULayer;
  this->layers[42]->setName("res4e_branch2a_activation_35_relu");
  this->layers[43] = new MWFusedConvReLULayer;
  this->layers[43]->setName("res4e_branch2b_activation_36_relu");
  this->layers[44] = new MWFusedConvReLULayer;
  this->layers[44]->setName("res4e_branch2c_activation_37_relu");
  this->layers[44]->setInPlaceIndex(0, 1);
  this->layers[45] = new MWFusedConvReLULayer;
  this->layers[45]->setName("res4f_branch2a_activation_38_relu");
  this->layers[46] = new MWFusedConvReLULayer;
  this->layers[46]->setName("res4f_branch2b_activation_39_relu");
  this->layers[47] = new MWFusedConvReLULayer;
  this->layers[47]->setName("res4f_branch2c_activation_40_relu");
  this->layers[47]->setInPlaceIndex(0, 1);
  this->layers[48] = new MWFusedConvReLULayer;
  this->layers[48]->setName("yolov2Conv1_yolov2Relu1");
  this->layers[49] = new MWFusedConvReLULayer;
  this->layers[49]->setName("yolov2Conv2_yolov2Relu2");
  this->layers[50] = new MWConvLayer;
  this->layers[50]->setName("yolov2ClassConv");
  this->layers[51] = new MWYoloExtractionLayer;
  this->layers[51]->setName("YOLOv2ExtractionLayer");
  this->layers[52] = new MWSigmoidLayer;
  this->layers[52]->setName("YOLOSigmoidLayer");
  this->layers[53] = new MWExponentialLayer;
  this->layers[53]->setName("YOLOv2ExponentialLayer");
  this->layers[54] = new MWYoloSoftmaxLayer;
  this->layers[54]->setName("YOLOv2SoftmaxLayer");
  this->layers[55] = new MWConcatenationLayer;
  this->layers[55]->setName("YOLOv2ConcatenationLayer");
  this->layers[56] = new MWOutputLayer;
  this->layers[56]->setName("yolov2OutputLayer");
  this->layers[56]->setInPlaceIndex(0, 0);
  this->targetImpl = new MWTargetNetworkImpl;
  this->inputTensors[0] = new MWTensor<real32_T>;
  this->inputTensors[0]->setHeight(224);
  this->inputTensors[0]->setWidth(224);
  this->inputTensors[0]->setChannels(3);
  this->inputTensors[0]->setBatchSize(1);
  this->inputTensors[0]->setSequenceLength(1);
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::deallocate()
{
  int32_T c5_idx;
  this->targetImpl->deallocate();
  for (c5_idx = 0; c5_idx < 57; c5_idx++) {
    this->layers[c5_idx]->deallocate();
  }
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::setSize()
{
  int32_T c5_idx;
  for (c5_idx = 0; c5_idx < 57; c5_idx++) {
    this->layers[c5_idx]->propagateSize();
  }

  this->allocate();
  this->postsetup();
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::resetState()
{
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::setup()
{
  if (this->isInitialized) {
    this->resetState();
  } else {
    this->isInitialized = true;
    this->targetImpl->preSetup();
    this->targetImpl->setAutoTune(true);
    (static_cast<MWInputLayer *>(this->layers[0]))->createInputLayer
      (this->targetImpl, this->inputTensors[0], 224, 224, 3, 0, "", 0);
    (static_cast<MWElementwiseAffineLayer *>(this->layers[1]))
      ->createElementwiseAffineLayer(this->targetImpl, this->layers[0]
      ->getOutputTensor(0), 1, 1, 3, 1, 1, 3, false, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_input_1_scale.bi"
      "n",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_input_1_offset.b"
      "in", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[2]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[1]
      ->getOutputTensor(0), 7, 7, 3, 64, 2, 2, 3, 3, 3, 3, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_conv1_w.bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_conv1_b.bin",
      1);
    (static_cast<MWMaxPoolingLayer *>(this->layers[3]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[2]->getOutputTensor(0), 3, 3, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[4]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[3]
      ->getOutputTensor(0), 1, 1, 64, 64, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2a_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2a_branch2a_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[5]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[4]
      ->getOutputTensor(0), 3, 3, 64, 64, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2a_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2a_branch2b_b"
      ".bin", 2);
    (static_cast<MWConvLayer *>(this->layers[6]))->createConvLayer
      (this->targetImpl, this->layers[3]->getOutputTensor(0), 1, 1, 64, 256, 1,
       1, 0, 0, 0, 0, 1, 1, 1,
       "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2a_branch1_w."
       "bin",
       "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2a_branch1_b."
       "bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[7]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[5]
      ->getOutputTensor(0), this->layers[6]->getOutputTensor(0), 1, 1, 64, 256,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2a_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2a_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[8]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[7]
      ->getOutputTensor(0), 1, 1, 256, 64, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2b_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2b_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[9]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[8]
      ->getOutputTensor(0), 3, 3, 64, 64, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2b_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2b_branch2b_b"
      ".bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[10]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[9]
      ->getOutputTensor(0), this->layers[7]->getOutputTensor(0), 1, 1, 64, 256,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2b_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2b_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[11]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[10]
      ->getOutputTensor(0), 1, 1, 256, 64, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2c_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2c_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[12]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[11]
      ->getOutputTensor(0), 3, 3, 64, 64, 2, 2, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2c_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2c_branch2b_b"
      ".bin", 2);
    (static_cast<MWMaxPoolingLayer *>(this->layers[13]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[10]->getOutputTensor(0), 1, 1, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[14]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[12]
      ->getOutputTensor(0), this->layers[13]->getOutputTensor(0), 1, 1, 64, 256,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2c_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res2c_branch2c_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[15]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[14]
      ->getOutputTensor(0), 1, 1, 256, 128, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3a_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3a_branch2a_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[16]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[15]
      ->getOutputTensor(0), 3, 3, 128, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3a_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3a_branch2b_b"
      ".bin", 2);
    (static_cast<MWConvLayer *>(this->layers[17]))->createConvLayer
      (this->targetImpl, this->layers[14]->getOutputTensor(0), 1, 1, 256, 512, 1,
       1, 0, 0, 0, 0, 1, 1, 1,
       "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3a_branch1_w."
       "bin",
       "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3a_branch1_b."
       "bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[18]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[16]
      ->getOutputTensor(0), this->layers[17]->getOutputTensor(0), 1, 1, 128, 512,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3a_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3a_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[19]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[18]
      ->getOutputTensor(0), 1, 1, 512, 128, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3b_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3b_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[20]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[19]
      ->getOutputTensor(0), 3, 3, 128, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3b_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3b_branch2b_b"
      ".bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[21]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[20]
      ->getOutputTensor(0), this->layers[18]->getOutputTensor(0), 1, 1, 128, 512,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3b_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3b_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[22]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[21]
      ->getOutputTensor(0), 1, 1, 512, 128, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3c_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3c_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[23]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[22]
      ->getOutputTensor(0), 3, 3, 128, 128, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3c_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3c_branch2b_b"
      ".bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[24]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[23]
      ->getOutputTensor(0), this->layers[21]->getOutputTensor(0), 1, 1, 128, 512,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3c_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3c_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[25]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[24]
      ->getOutputTensor(0), 1, 1, 512, 128, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3d_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3d_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[26]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[25]
      ->getOutputTensor(0), 3, 3, 128, 128, 2, 2, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3d_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3d_branch2b_b"
      ".bin", 2);
    (static_cast<MWMaxPoolingLayer *>(this->layers[27]))->createMaxPoolingLayer
      (this->targetImpl, this->layers[24]->getOutputTensor(0), 1, 1, 2, 2, 0, 0,
       0, 0, 0, 1, 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[28]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[26]
      ->getOutputTensor(0), this->layers[27]->getOutputTensor(0), 1, 1, 128, 512,
      1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3d_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res3d_branch2c_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[29]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[28]
      ->getOutputTensor(0), 1, 1, 512, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4a_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4a_branch2a_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[30]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[29]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4a_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4a_branch2b_b"
      ".bin", 2);
    (static_cast<MWConvLayer *>(this->layers[31]))->createConvLayer
      (this->targetImpl, this->layers[28]->getOutputTensor(0), 1, 1, 512, 1024,
       1, 1, 0, 0, 0, 0, 1, 1, 1,
       "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4a_branch1_w."
       "bin",
       "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4a_branch1_b."
       "bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[32]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[30]
      ->getOutputTensor(0), this->layers[31]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4a_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4a_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[33]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[32]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4b_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4b_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[34]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[33]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4b_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4b_branch2b_b"
      ".bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[35]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[34]
      ->getOutputTensor(0), this->layers[32]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4b_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4b_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[36]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[35]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4c_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4c_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[37]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[36]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4c_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4c_branch2b_b"
      ".bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[38]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[37]
      ->getOutputTensor(0), this->layers[35]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4c_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4c_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[39]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[38]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4d_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4d_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[40]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[39]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4d_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4d_branch2b_b"
      ".bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[41]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[40]
      ->getOutputTensor(0), this->layers[38]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4d_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4d_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[42]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[41]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4e_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4e_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[43]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[42]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4e_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4e_branch2b_b"
      ".bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[44]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[43]
      ->getOutputTensor(0), this->layers[41]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4e_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4e_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[45]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[44]
      ->getOutputTensor(0), 1, 1, 1024, 256, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4f_branch2a_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4f_branch2a_b"
      ".bin", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[46]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[45]
      ->getOutputTensor(0), 3, 3, 256, 256, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4f_branch2b_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4f_branch2b_b"
      ".bin", 2);
    (static_cast<MWFusedConvReLULayer *>(this->layers[47]))
      ->createFusedConvReLULayer(this->targetImpl, 2, this->layers[46]
      ->getOutputTensor(0), this->layers[44]->getOutputTensor(0), 1, 1, 256,
      1024, 1, 1, 0, 0, 0, 0, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4f_branch2c_w"
      ".bin",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_res4f_branch2c_b"
      ".bin", 1);
    (static_cast<MWFusedConvReLULayer *>(this->layers[48]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[47]
      ->getOutputTensor(0), 3, 3, 1024, 1024, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_yolov2Conv1_w.bi"
      "n",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_yolov2Conv1_b.bi"
      "n", 0);
    (static_cast<MWFusedConvReLULayer *>(this->layers[49]))
      ->createFusedConvReLULayer(this->targetImpl, 1, this->layers[48]
      ->getOutputTensor(0), 3, 3, 1024, 1024, 1, 1, 1, 1, 1, 1, 1, 1, 1,
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_yolov2Conv2_w.bi"
      "n",
      "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_yolov2Conv2_b.bi"
      "n", 1);
    (static_cast<MWConvLayer *>(this->layers[50]))->createConvLayer
      (this->targetImpl, this->layers[49]->getOutputTensor(0), 1, 1, 1024, 24, 1,
       1, 0, 0, 0, 0, 1, 1, 1,
       "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_yolov2ClassConv_"
       "w.bin",
       "/home/hmcl/ADcontrol/slprj/_sfprj/LaneDetection/_self/sfun/src/cnn_yolov2ResNet50VehicleExample0_LaneDetection0_yolov2ClassConv_"
       "b.bin", 0);
    (static_cast<MWYoloExtractionLayer *>(this->layers[51]))
      ->createYoloExtractionLayer(this->targetImpl, this->layers[50]
      ->getOutputTensor(0), 4, 1, 2, 3);
    (static_cast<MWSigmoidLayer *>(this->layers[52]))->createSigmoidLayer
      (this->targetImpl, this->layers[51]->getOutputTensor(0), 0);
    (static_cast<MWExponentialLayer *>(this->layers[53]))
      ->createExponentialLayer(this->targetImpl, this->layers[51]
      ->getOutputTensor(1), 1);
    (static_cast<MWYoloSoftmaxLayer *>(this->layers[54]))
      ->createYoloSoftmaxLayer(this->targetImpl, this->layers[51]
      ->getOutputTensor(2), 4, 2);
    (static_cast<MWConcatenationLayer *>(this->layers[55]))
      ->createConcatenationLayer(this->targetImpl, 3, this->layers[52]
      ->getOutputTensor(0), this->layers[53]->getOutputTensor(0), this->layers
      [54]->getOutputTensor(0), 3, 3);
    (static_cast<MWOutputLayer *>(this->layers[56]))->createOutputLayer
      (this->targetImpl, this->layers[55]->getOutputTensor(0), 3);
    this->outputTensors[0] = this->layers[56]->getOutputTensor(0);
    this->setSize();
  }
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::predict()
{
  int32_T c5_idx;
  for (c5_idx = 0; c5_idx < 57; c5_idx++) {
    this->layers[c5_idx]->predict();
  }
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::activations(int32_T
  c5_layerIdx)
{
  int32_T c5_idx;
  for (c5_idx = 0; c5_idx <= c5_layerIdx; c5_idx++) {
    this->layers[c5_idx]->predict();
  }
}

void c5_yolov2ResNet50VehicleExample0_LaneDetection0::cleanup()
{
  int32_T c5_idx;
  this->deallocate();
  for (c5_idx = 0; c5_idx < 57; c5_idx++) {
    this->layers[c5_idx]->cleanup();
  }

  if (this->targetImpl) {
    this->targetImpl->cleanup();
  }
}

real32_T *c5_yolov2ResNet50VehicleExample0_LaneDetection0::getLayerOutput
  (int32_T c5_layerIndex, int32_T c5_portIndex)
{
  return this->layers[c5_layerIndex]->getLayerOutput(c5_portIndex);
}

real32_T *c5_yolov2ResNet50VehicleExample0_LaneDetection0::getInputDataPointer
  (int32_T c5_index)
{
  return (static_cast<MWTensor<real32_T> *>(this->inputTensors[c5_index]))
    ->getData();
}

real32_T *c5_yolov2ResNet50VehicleExample0_LaneDetection0::getInputDataPointer()
{
  return (static_cast<MWTensor<real32_T> *>(this->inputTensors[0]))->getData();
}

real32_T *c5_yolov2ResNet50VehicleExample0_LaneDetection0::getOutputDataPointer
  (int32_T c5_index)
{
  return (static_cast<MWTensor<real32_T> *>(this->outputTensors[c5_index]))
    ->getData();
}

real32_T *c5_yolov2ResNet50VehicleExample0_LaneDetection0::getOutputDataPointer()
{
  return (static_cast<MWTensor<real32_T> *>(this->outputTensors[0]))->getData();
}

int32_T c5_yolov2ResNet50VehicleExample0_LaneDetection0::getBatchSize()
{
  return this->inputTensors[0]->getBatchSize();
}

c5_yolov2ResNet50VehicleExample0_LaneDetection0::
  ~c5_yolov2ResNet50VehicleExample0_LaneDetection0()
{
  int32_T c5_idx;
  this->cleanup();
  c5_checkCleanupCudaError(cudaGetLastError(), __FILE__, __LINE__);
  for (c5_idx = 0; c5_idx < 57; c5_idx++) {
    delete this->layers[c5_idx];
  }

  if (this->targetImpl) {
    delete this->targetImpl;
  }

  delete this->inputTensors[0];
}

static void c5_checkCleanupCudaError(cudaError_t c5_errCode, const char_T
  *c5_file, uint32_T c5_line)
{
  emlrtRTEInfo c5_rtInfo;
  if ((c5_errCode != cudaSuccess) && (c5_errCode != cudaErrorCudartUnloading)) {
    c5_rtInfo = c5_createEmlrtInfoStruct(c5_file, c5_line);
    emlrtCUDAWarning(c5_errCode, cudaGetErrorName(c5_errCode),
                     cudaGetErrorString(c5_errCode), &c5_rtInfo);
  }
}

static emlrtRTEInfo c5_createEmlrtInfoStruct(const char_T *c5_file, uint32_T
  c5_line)
{
  emlrtRTEInfo c5_b_rtInfo;
  uint32_T c5_len;
  char_T *c5_brk;
  char_T *c5_fn;
  char_T *c5_pn;
  c5_len = (uint32_T)strlen(c5_file);
  c5_pn = (char_T *)calloc(c5_len + 1U, 1U);
  c5_fn = (char_T *)calloc(c5_len + 1U, 1U);
  memcpy(c5_pn, c5_file, c5_len);
  memcpy(c5_fn, c5_file, c5_len);
  c5_brk = strrchr(c5_fn, '.');
  *c5_brk = '\x00';
  c5_brk = NULL;
  c5_brk = strrchr(c5_fn, '/');
  if (c5_brk == NULL) {
    c5_brk = strrchr(c5_fn, '\\');
  }

  if (c5_brk == NULL) {
    c5_brk = c5_fn;
  } else {
    c5_brk++;
  }

  c5_b_rtInfo.lineNo = c5_line;
  c5_b_rtInfo.colNo = 0;
  c5_b_rtInfo.fName = c5_brk;
  c5_b_rtInfo.pName = c5_pn;
  return c5_b_rtInfo;
}

static void init_dsm_address_info(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static void init_simulink_io_address(SFc5_LaneDetectionInstanceStruct
  *chartInstance)
{
  chartInstance->c5_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c5_bboxes_data = (real_T (*)[80])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c5_bboxes_sizes = (int32_T (*)[2])
    ssGetCurrentOutputPortDimensions_wrapper(chartInstance->S, 1);
  chartInstance->c5_In = (real32_T (*)[921600])ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c5_scores_data = (real32_T (*)[20])
    ssGetOutputPortSignal_wrapper(chartInstance->S, 2);
  chartInstance->c5_scores_sizes = (int32_T (*)[2])
    ssGetCurrentOutputPortDimensions_wrapper(chartInstance->S, 2);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c5_LaneDetection_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(955709936U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3991420490U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2490454663U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2264674843U);
}

mxArray *sf_c5_LaneDetection_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,1);
  mxSetCell(mxcell3p, 0, mxCreateString("dltargets.cudnn.cudnnApi"));
  return(mxcell3p);
}

mxArray *sf_c5_LaneDetection_jit_fallback_info(void)
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

mxArray *sf_c5_LaneDetection_updateBuildInfo_args_info(void)
{
  mxArray *mxBIArgs = sf_mex_decode(
    "eNrlWMFqwkAQjaUthYJWQatYEASvfkOLVSrVUGpL21NZklWWJrsh2Wh667Gf18/osccem0QTN2N"
    "q8CRsAiHsMtnHy8y8mYlSGI4V/yr691dNUY7954l/F5TldSSsz4X9wP5X2W5/AOwvBfvDFPuiYH"
    "+2WiNdnzDX1vCAGNhZ7n1k4BYA7l0G7gXADdbjp76BTUz5gjj4ajolFI/QO7a7mmVF5/7sib+3I"
    "/+bDNw6wK2H/Aeug/Ueo/N7PHqMuQfnfe+Jt7Uj7yzcU4AbrDVKX5FFIjdL5ecywC2Hfu6paujd"
    "oWkZXc2VML9bALe1Jb+jrxCcG7y0D/6fO/J/zsDtANxOBv9bbFO8igWZ4qAJcJv/6JyYCTLlfw3"
    "g1kL+D8ieYa5ivmD2W0xdJr83AG5jqXuuw5kZOnzA7J57raqx6uehvvshryGOKeKEUSnrewngli"
    "Le83UrJ39+9z2LUV/oCTKEDlYm3hWAWwl5T8jMZERPdO150LUXZrC+x22kJRJbQl1Lj/eA/4RNu"
    "Yk8EO9y9zGbei5rH5Oe77GuJwYZ+fMd6nvu5rgU/usJRqa4rwLc6kadW/te/v9zKXVOCH2Z5vc2"
    "wG1v4y8O77npd8R6n9S/P7KUnlA="
    );
  return mxBIArgs;
}

static const mxArray *sf_get_sim_state_info_c5_LaneDetection(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNpjYPT0ZQACPiA+wcTAwAakOYCYiQECWKF8RiBmhtIQcRa4uAIQl1QWpILEi4uSPVOAdF5iLpi"
    "fWFrhmZeWDzbfggFhPhsW8xmRzOeEikPAB3vK9Is4oOtnwaKfDUm/AJSflJRfkVoMCR9YOA2cPx"
    "TI9kdxcn4R9fyB6Q7S9EPsDyDgDyk0f4D4mcXxicklmWWp8cmm8T6JeakuqSWpQIH8PIS5IAAA5"
    "lYhew=="
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c5_LaneDetection_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "ssN2SKAuylfcyN4by9KRKKE";
}

static void sf_opaque_initialize_c5_LaneDetection(void *chartInstanceVar)
{
  initialize_params_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
    chartInstanceVar);
  initialize_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c5_LaneDetection(void *chartInstanceVar)
{
  enable_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c5_LaneDetection(void *chartInstanceVar)
{
  disable_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c5_LaneDetection(void *chartInstanceVar)
{
  sf_gateway_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c5_LaneDetection(SimStruct* S)
{
  return get_sim_state_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c5_LaneDetection(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_cleanup_runtime_resources_c5_LaneDetection(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc5_LaneDetectionInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_LaneDetection_optimization_info();
    }

    mdl_cleanup_runtime_resources_c5_LaneDetection
      ((SFc5_LaneDetectionInstanceStruct*) chartInstanceVar);
    ((SFc5_LaneDetectionInstanceStruct*) chartInstanceVar)->
      ~SFc5_LaneDetectionInstanceStruct();
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c5_LaneDetection(void *chartInstanceVar)
{
  mdl_start_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_mdl_terminate_c5_LaneDetection(void *chartInstanceVar)
{
  mdl_terminate_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c5_LaneDetection(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  sf_warn_if_symbolic_dimension_param_changed(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
      sf_get_chart_instance_ptr(S));
    initSimStructsc5_LaneDetection((SFc5_LaneDetectionInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

const char* sf_c5_LaneDetection_get_post_codegen_info(void)
{
  int i;
  const char* encStrCodegen [32] = {
    "eNrtXM1v40QUT7vLipVgtQcE0grE3kBCIm2SIpAQbJsPEfVjS9Mtx2hmPKmH2GPXM04a7Yn9+D8",
    "47pEDB/4EjnBA4j9A3Lhw542Tpq5jJ+t2afKijZSmdt48/97nvBk/p7DS3C3A6w68X/BC4RZ8vg",
    "nv1cLw9cboeCX2Hp6/WfhodPwzDJKhu08C4qrC1JckLj/gynNCLTzZlB0vlUzIDg+4ZEDre4HO4",
    "qaEGzpCdhuhZIaf+s4WzG7ZXuhYWzCWWA+lMwBufqj3gU9NBJzpBueWtgMvPLYbDjkeIw50v2pz",
    "1lWhO00ExXUr9A0stRs6WvgOr59y1pRKE0CszrG1NNG8qk8zxTSSqtYZoef6jiAyVVqbqBb3QcG",
    "aP/It+Psw1CBUkozZJNBb3CY9rnZEN+LpSZ7kKRR8QYUk2gsEcequUzUDJ7HtO4Bn17O4M0UhgG",
    "0r4KTre0LqbPu3GiBpXRLq8Bqn4XE2txY/CY3xjwTv8yBTb52q1+MBOeYPZeZFI4XUTyNrjb1kk",
    "kwLlx+RYJOB/RS3Mr0XPEe1CNiJH8KILDIeCdlUh4HogXozuYVu03jmrJAJ3aGx1SyyiFu9x6dZ",
    "YcytwWSVOI7KJDv0/B3e407EtUY0mU425JpOp5SwDj1QsHHv7GgIpQDDj8iqnrREqrl6CYIo7+x",
    "BYrlIyUKlPbcKzlvb2Zn8epKsKTUPOoTxtCwQEKE46CxSbzY3SyhjeyAEVDqCl0Y89JBZVAXVCW",
    "Wt7wVd0MmUJHIugrFoJqGrjsGWEAmPFATNNDJjy1l0jDCbWybBCIfvQtgAbYpOlEltmxB3PaEHN",
    "a5YIPwUq4YQdZCG6sahBj5/JLvS68tG4LmtUY4fWgEyA+RwF2xwGMWYZMBKKA3pQpxf3uIcnJIE",
    "UsjjLUhzwaABIFMtZua9tcL5vPfWS8x7Z+OSnx/H+Kyk8CnEPpPXvb06/bqr8N/KaNyD2Li3E9e",
    "5mRhn6O7C++gP9sHWb+TfL//6/cW3737y91Wuf3ojX51wZ3T8/llCHgdYb8KvDe03MVw3U/i/F+",
    "N/d3Ss1F6ptb0ZDpwOG+xV6OCL7YPt7XrE79fV6XhvJPCenb9vZgbwxsiPA9a0RgWMOSbhcFo3/",
    "D+P4b01Qx+3R+eHr3++vtr4dx4kx6fp61ZCX+aYUu/URETcf+cnx/1Ly6GYF7w6OSZx5Bs/vP7+",
    "DDnuJeS4F9UzbWKyJG+zjfYOkVAjaR6VK5N5Jm/cvh73etwij1u55Dx82XGrV5z3r2vcVeXLW48",
    "sGv3alDxaSNDfXWA5rlon/t/0fxby1XMfjo6/Gi/tqrZwrJQqf/Q11OKdtG+XxE/9g+n0foL+lx",
    "n1wQ8JvzbHRdtzedF2mVPcrMGiRweeU1SOH3xfbKuO+bhQNMBJ7nSKZgFZhNK1yKRsDzzH65UOu",
    "NrjemPtiNuCma2raDtj7WLNsTYihjV2r9Smn1Ih0eJfb/dz4n+ewP98fvgdopQRYiwDVvw0B/5n",
    "CfzP5oAfFhWVTpsGRDK7xM5dCCl+ihE/R67/MX6KHj9u/yHI4xe7/xCc/mMhz58Wcv+3kPsPQz5",
    "/MeT+z5bF/yl6/aPET5HHL0UevxR5/UNxx2+J4s6fcfwo9U+WRf+J+P1xBn4vgd97Nfh1QITklq",
    "EDCSYQd5g5sT5yFUx6ZsjzJLlknD5N4H865zhdj+2UL7qfl/D5eZlkzKc/zcD/OIH/8Rzws8SdC",
    "JTzUM46ePHiE7n+sa8DKfI6LOc66kkC/5M54I+efWivt71OR3F9fiMUZd7HWceXybLcR0S6j0OQ",
    "74N0MvLnrPrSTeB3r6W+ZBd6RjDF6WX3ixeh3+UszytGHJ7od1lcP6nk8pNF64vqo9FzOVf/0KL",
    "EI12WeR/n/l3ZuuT+3fz8fOOCny/y/IgwHkvI74eVreXZT6d49tP7433GRZ4fKYq8sYGyrqbL0g",
    "eDZh5cx+gnpax+BTz7+xTj/nKZLcv+CMESn5UL9d+LGThPEjhPrgWnRTSZ2DZ+Kb12E3i711Rvf",
    "Jbz/uv8cPax54k+8udJKPJ+UJTrF+T9oCXk/cTlpdE/RY6focePsQ+kktWnhQR/eXnw46x/CO75",
    "t4z8PngZeT96GfnzAPHnIfuo+7go8vqNIo9f/PrHfv+LYutvif+eS97fo1nA53ko8n4A3PmH5bt",
    "/6ifw+9e3/zpu50LpJwy5n6Nf56JcZ1WQ901VCMI+jVKsTwOhnnE/H07w9BUg7FurkGX5HYdY3f",
    "gf5Lio9w==",
    ""
  };

  static char newstr [2261] = "";
  newstr[0] = '\0';
  for (i = 0; i < 32; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c5_LaneDetection(SimStruct *S)
{
  const char* newstr = sf_c5_LaneDetection_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(493081942U));
  ssSetChecksum1(S,(4116828226U));
  ssSetChecksum2(S,(2832328508U));
  ssSetChecksum3(S,(3911915089U));
}

static void mdlRTW_c5_LaneDetection(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c5_LaneDetection(SimStruct *S)
{
  SFc5_LaneDetectionInstanceStruct *chartInstance;
  chartInstance = (SFc5_LaneDetectionInstanceStruct *)utMalloc(sizeof
    (SFc5_LaneDetectionInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc5_LaneDetectionInstanceStruct));
  chartInstance = new (chartInstance) SFc5_LaneDetectionInstanceStruct;
  chartInstance->chartInfo.chartInstance = chartInstance;
  if (ssGetSampleTime(S, 0) == CONTINUOUS_SAMPLE_TIME && ssGetOffsetTime(S, 0) ==
      0 && ssGetNumContStates(ssGetRootSS(S)) > 0) {
    sf_error_out_about_continuous_sample_time_with_persistent_vars(S);
  }

  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c5_LaneDetection;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c5_LaneDetection;
  chartInstance->chartInfo.mdlStart = sf_opaque_mdl_start_c5_LaneDetection;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c5_LaneDetection;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c5_LaneDetection;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c5_LaneDetection;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c5_LaneDetection;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c5_LaneDetection;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c5_LaneDetection;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c5_LaneDetection;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c5_LaneDetection;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c5_LaneDetection;
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
    chartInstance->c5_JITStateAnimation,
    chartInstance->c5_JITTransitionAnimation);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c5_LaneDetection(chartInstance);
}

void c5_LaneDetection_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c5_LaneDetection(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c5_LaneDetection(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c5_LaneDetection(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c5_LaneDetection_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
