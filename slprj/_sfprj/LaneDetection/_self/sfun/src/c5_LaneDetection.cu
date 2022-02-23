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
  real32_T c5_b_In[1229760], real_T c5_b_bboxes_data[], int32_T c5_bboxes_size[2],
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
static __global__ void c5_eML_blk_kernel_kernel2(int16_T c5_aux2[1708]);
static __global__ void c5_eML_blk_kernel_kernel3(const int16_T c5_aux1[960],
  real_T c5_rowWeights[2016], int16_T c5_ipRowIndices[2016]);
static __global__ void c5_eML_blk_kernel_kernel4(const int16_T c5_aux2[1708],
  real_T c5_colWeights[3584], int16_T c5_ipColIndices[3584]);
static __global__ void c5_eML_blk_kernel_kernel5(const real_T c5_rowWeights[2016],
  real_T c5_rowWeightsTotal[224]);
static __global__ void c5_eML_blk_kernel_kernel6(const real_T c5_rowWeights[2016],
  const int32_T c5_xoffset, real_T c5_rowWeightsTotal[224]);
static __global__ void c5_eML_blk_kernel_kernel7(const real_T c5_colWeights[3584],
  real_T c5_colWeightsTotal[224]);
static __global__ void c5_eML_blk_kernel_kernel8(const real_T c5_colWeights[3584],
  const int32_T c5_xoffset, real_T c5_colWeightsTotal[224]);
static __global__ void c5_eML_blk_kernel_kernel9(const real_T
  c5_colWeightsTotal[224], const real_T c5_colWeights[3584], const real32_T
  c5_b_In[1229760], const int16_T c5_ipColIndices[3584], real32_T
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
  cudaMalloc(&chartInstance->c5_h_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_scores_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_out, 602112UL);
  cudaMalloc(&chartInstance->c5_gpu_x1_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_bv, 784UL);
  cudaMalloc(&chartInstance->c5_gpu_x2_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_area_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_y1_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_i38, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_out_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_ipRowIndices, 4032UL);
  cudaMalloc(&chartInstance->c5_c_gpu_end, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_x1_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_outVal, 8UL);
  cudaMalloc(&chartInstance->c5_b_gpu_bboxPred_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_rowWeightsTotal, 1792UL);
  cudaMalloc(&chartInstance->c5_gpu_i35, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_In, 4919040UL);
  cudaMalloc(&chartInstance->c5_gpu_dv, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_ii_data, 1568UL);
  cudaMalloc(&chartInstance->c5_gpu_anchors, 64UL);
  cudaMalloc(&chartInstance->c5_gpu_i61, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_scorePred_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_thresholdedPrediction_data, 18816UL);
  cudaMalloc(&chartInstance->c5_gpu_count, 4UL);
  cudaMalloc(&chartInstance->c5_c_gpu_n, 4UL);
  cudaMalloc(&chartInstance->c5_c_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_ii_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_b_data, 784UL);
  cudaMalloc(&chartInstance->c5_gpu_n, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_out, 602112UL);
  cudaMalloc(&chartInstance->c5_gpu_selectedIndex_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i14, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_ipColIndices, 7168UL);
  cudaMalloc(&chartInstance->c5_gpu_classPred_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i12, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i10, 4UL);
  cudaMalloc(&chartInstance->c5_d_gpu_end, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_thresholdedPrediction_size, 8UL);
  cudaMalloc(&chartInstance->c5_d_gpu_bboxPred_size, 8UL);
  cudaMalloc(&chartInstance->c5_b_gpu_n, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i17, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_y2_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_end, 4UL);
  cudaMalloc(&chartInstance->c5_c_gpu_bboxPred_data, 25088UL);
  cudaMalloc(&chartInstance->c5_gpu_b_size, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_i4, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_rowWeights, 16128UL);
  cudaMalloc(&chartInstance->c5_gpu_partialResize, 1290240UL);
  cudaMalloc(&chartInstance->c5_d_gpu_bboxPred_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_i, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i50, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_iv1_data, 1568UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxPred_size, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_aux1, 1920UL);
  cudaMalloc(&chartInstance->c5_gpu_xoffset, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i47, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_iv1_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_dv1, 32UL);
  cudaMalloc(&chartInstance->c5_b_gpu_bboxPred_data, 25088UL);
  cudaMalloc(&chartInstance->c5_gpu_i9, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxPred_data, 6272UL);
  cudaMalloc(&chartInstance->c5_gpu_idx_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_index_data, 784UL);
  cudaMalloc(&chartInstance->c5_d_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_count, 8UL);
  cudaMalloc(&chartInstance->c5_b_gpu_outVal, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i66, 4UL);
  cudaMalloc(&chartInstance->c5_e_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_nrows, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i7, 4UL);
  cudaMalloc(&chartInstance->c5_g_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_colWeights, 28672UL);
  cudaMalloc(&chartInstance->c5_gpu_scores_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_nx, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxes_data, 80U * sizeof(real_T));
  cudaMalloc(&chartInstance->c5_gpu_classPred_data, 3136UL);
  cudaMalloc(&chartInstance->c5_gpu_i64, 4UL);
  cudaMalloc(&chartInstance->c5_f_gpu_loop_ub, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i32, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i19, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_aux2, 3416UL);
  cudaMalloc(&chartInstance->c5_e_gpu_bboxPred_data, 25088UL);
  cudaMalloc(&chartInstance->c5_gpu_i3, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i51, 4UL);
  cudaMalloc(&chartInstance->c5_b_gpu_xoffset, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_dv2, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_scorePred_data, 3136UL);
  cudaMalloc(&chartInstance->c5_b_gpu_end, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_colWeightsTotal, 1792UL);
  cudaMalloc(&chartInstance->c5_gpu_selectedIndex_data, 784UL);
  cudaMalloc(&chartInstance->c5_gpu_boxOut, 18816UL);
  cudaMalloc(&chartInstance->c5_gpu_y, 4UL);
  cudaMalloc(&chartInstance->c5_c_gpu_bboxPred_size, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_iv, 8UL);
  cudaMalloc(&chartInstance->c5_b_gpu_anchors, 32UL);
  cudaMalloc(&chartInstance->c5_e_gpu_bboxPred_size, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxesX1Y1X2Y2_size, 8UL);
  cudaMalloc(&chartInstance->c5_gpu_tmpFeatureMap, 18816UL);
  cudaMalloc(&chartInstance->c5_b_gpu_scores_data, 20U * sizeof(real32_T));
  cudaMalloc(&chartInstance->c5_gpu_i39, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i45, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_i6, 4UL);
  cudaMalloc(&chartInstance->c5_gpu_bboxesX1Y1X2Y2_data, 25088UL);
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
  for (c5_i = 0; c5_i < 1229760; c5_i++) {
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
  cudaFree(*chartInstance->c5_gpu_aux1);
  cudaFree(*chartInstance->c5_gpu_iv);
  cudaFree(*chartInstance->c5_gpu_index_data);
  cudaFree(chartInstance->c5_gpu_i39);
  cudaFree(*chartInstance->c5_gpu_aux2);
  cudaFree(*chartInstance->c5_gpu_In);
  cudaFree(chartInstance->c5_gpu_i32);
  cudaFree(*chartInstance->c5_b_gpu_out);
  cudaFree(chartInstance->c5_g_gpu_loop_ub);
  cudaFree(*chartInstance->c5_gpu_iv1_size);
  cudaFree(*chartInstance->c5_e_gpu_bboxPred_size);
  cudaFree(chartInstance->c5_f_gpu_loop_ub);
  cudaFree(*chartInstance->c5_gpu_rowWeightsTotal);
  cudaFree(*chartInstance->c5_gpu_scorePred_size);
  cudaFree(chartInstance->c5_h_gpu_loop_ub);
  cudaFree(*chartInstance->c5_gpu_bboxPred_data);
  cudaFree(*chartInstance->c5_gpu_rowWeights);
  cudaFree(*chartInstance->c5_c_gpu_bboxPred_size);
  cudaFree(*chartInstance->c5_gpu_tmpFeatureMap);
  cudaFree(*chartInstance->c5_gpu_selectedIndex_size);
  cudaFree(chartInstance->c5_gpu_i64);
  cudaFree(*chartInstance->c5_e_gpu_bboxPred_data);
  cudaFree(*chartInstance->c5_gpu_selectedIndex_data);
  cudaFree(chartInstance->c5_gpu_i38);
  cudaFree(*chartInstance->c5_gpu_classPred_size);
  cudaFree(*chartInstance->c5_d_gpu_bboxPred_data);
  cudaFree(chartInstance->c5_gpu_i9);
  cudaFree(*chartInstance->c5_gpu_ii_size);
  cudaFree(chartInstance->c5_gpu_end);
  cudaFree(*chartInstance->c5_gpu_thresholdedPrediction_data);
  cudaFree(*chartInstance->c5_gpu_colWeights);
  cudaFree(chartInstance->c5_gpu_i);
  cudaFree(*chartInstance->c5_d_gpu_bboxPred_size);
  cudaFree(chartInstance->c5_b_gpu_count);
  cudaFree(*chartInstance->c5_gpu_colWeightsTotal);
  cudaFree(chartInstance->c5_gpu_nx);
  cudaFree(chartInstance->c5_gpu_n);
  cudaFree(chartInstance->c5_d_gpu_loop_ub);
  cudaFree(chartInstance->c5_gpu_i66);
  cudaFree(*chartInstance->c5_gpu_out_data);
  cudaFree(*chartInstance->c5_gpu_out);
  cudaFree(*chartInstance->c5_gpu_ipColIndices);
  cudaFree(chartInstance->c5_gpu_count);
  cudaFree(chartInstance->c5_gpu_i35);
  cudaFree(chartInstance->c5_gpu_i19);
  cudaFree(chartInstance->c5_gpu_nrows);
  cudaFree(*chartInstance->c5_gpu_scores_data);
  cudaFree(*chartInstance->c5_gpu_b_size);
  cudaFree(*chartInstance->c5_gpu_ipRowIndices);
  cudaFree(*chartInstance->c5_gpu_x1_data);
  cudaFree(*chartInstance->c5_gpu_ii_data);
  cudaFree(*chartInstance->c5_gpu_scores_size);
  cudaFree(*chartInstance->c5_gpu_dv);
  cudaFree(chartInstance->c5_b_gpu_loop_ub);
  cudaFree(chartInstance->c5_gpu_i12);
  cudaFree(*chartInstance->c5_gpu_partialResize);
  cudaFree(*chartInstance->c5_gpu_classPred_data);
  cudaFree(*chartInstance->c5_gpu_dv1);
  cudaFree(chartInstance->c5_gpu_i50);
  cudaFree(*chartInstance->c5_gpu_x1_size);
  cudaFree(chartInstance->c5_b_gpu_scores_data);
  cudaFree(chartInstance->c5_gpu_i14);
  cudaFree(chartInstance->c5_gpu_i6);
  cudaFree(chartInstance->c5_gpu_xoffset);
  cudaFree(*chartInstance->c5_gpu_dv2);
  cudaFree(*chartInstance->c5_gpu_thresholdedPrediction_size);
  cudaFree(*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data);
  cudaFree(chartInstance->c5_gpu_i7);
  cudaFree(*chartInstance->c5_gpu_bv);
  cudaFree(*chartInstance->c5_gpu_anchors);
  cudaFree(chartInstance->c5_b_gpu_outVal);
  cudaFree(*chartInstance->c5_gpu_scorePred_data);
  cudaFree(chartInstance->c5_c_gpu_n);
  cudaFree(*chartInstance->c5_c_gpu_bboxPred_data);
  cudaFree(*chartInstance->c5_gpu_idx_data);
  cudaFree(*chartInstance->c5_gpu_b_data);
  cudaFree(chartInstance->c5_gpu_i10);
  cudaFree(chartInstance->c5_gpu_i3);
  cudaFree(chartInstance->c5_gpu_loop_ub);
  cudaFree(chartInstance->c5_b_gpu_end);
  cudaFree(chartInstance->c5_d_gpu_end);
  cudaFree(chartInstance->c5_b_gpu_xoffset);
  cudaFree(chartInstance->c5_b_gpu_n);
  cudaFree(chartInstance->c5_c_gpu_end);
  cudaFree(chartInstance->c5_gpu_bboxes_data);
  cudaFree(*chartInstance->c5_gpu_bboxPred_size);
  cudaFree(chartInstance->c5_gpu_i47);
  cudaFree(chartInstance->c5_e_gpu_loop_ub);
  cudaFree(*chartInstance->c5_gpu_iv1_data);
  cudaFree(*chartInstance->c5_gpu_area_data);
  cudaFree(*chartInstance->c5_b_gpu_bboxPred_size);
  cudaFree(chartInstance->c5_c_gpu_loop_ub);
  cudaFree(*chartInstance->c5_gpu_outVal);
  cudaFree(*chartInstance->c5_b_gpu_anchors);
  cudaFree(chartInstance->c5_gpu_i61);
  cudaFree(chartInstance->c5_gpu_i4);
  cudaFree(*chartInstance->c5_b_gpu_bboxPred_data);
  cudaFree(*chartInstance->c5_gpu_y2_data);
  cudaFree(*chartInstance->c5_gpu_boxOut);
  cudaFree(chartInstance->c5_gpu_i51);
  cudaFree(chartInstance->c5_gpu_i45);
  cudaFree(*chartInstance->c5_gpu_y1_data);
  cudaFree(chartInstance->c5_gpu_i17);
  cudaFree(chartInstance->c5_gpu_y);
  cudaFree(*chartInstance->c5_gpu_bboxesX1Y1X2Y2_size);
  cudaFree(*chartInstance->c5_gpu_x2_data);
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
    "789ced9dc96f2bc97dc739c638193b9e19c59389c7419637882ff1244f24b550f225269ba4448a3b297179089e9acd22d962b3bbd90bb74b989b811c72c8c540"
    "f6c54e8c38f6047082acc835a75c72c97f906be0532e06d25c4a22fbb15f735464bd56f1d7c04cbfd6af55df5f2daa4fd7ee7b2b917ecbe7f3bd67fdf75fbfed",
    "f3fdc7d73fe79b5eeffae6d7c1e2fe39dfea65b7bfb5b8576dcff8fabcefed95df9bdaffd3d2fbbdc5b3a0c8061a1af30799efa2fbdf6c285d51e665a3345291"
    "4f43ba22f5516366698a122a895d545c7ec84c9fbaf125d3fdc3d434fd37d74642a768767d5a5b7ff0505a7e98a5c7f4ba7588efdb2ee961bfece9617f0feb4d",
    "1ea987c3ffaa8b1eb6f7515b1424f4b2810c2418a222bf1c2992f2b21fb4f9734be8cf4f39fa33b7e8a2dc92d083deb709f58e1cf556ed2f62bf75d856bae8b0"
    "dd15a4c370745afe34453a744c97e7ddd9efb9a5cbfb1bfa69bf3fbcffceecfea37f7ffb254d3d7ced8bded021bc4dcbd9cf3be81dd8ec6629d7e1e2289e4b5c",
    "c5cef944add82be75bb1073f722e3a6e7ef81c9e6985ffdf0ebfbf693af61cc23fb0d95f242ad65faca96b879222f0d2613a5c4a85238785a03fe8af1f1a8a22"
    "d595e1a1dee635d4386c4882d240dacb3aafa3d5874f66ff3efc44b468a3c9563892c237a208a929c46bb2551765903150b4ce73d5164fd2f2f2814b3cb17de6",
    "dff36a3695ed0717beacf8714be8c7dbb6e7073fe616c14ac165bdff25d4fb5d47bd55fb8b04b75c214715c1ec22d9d07146174d55553423c70b1dbe85f44d32"
    "5e9fff8a8a7f65fececb86f4d2e0b516b2c25e948695d45ee43cad7ae9d77ef231d4f33bd423fdbbfdd041efc066bf29b5ee7ae7c75757ed7845a8f74ffd8da3",
    "78d2c74e3dbfeff5c0b708e3ffb14bfcb1bdcb1b125fe72c7f5a48ce581fa5a6ccd72594d31415698688701b85767b61cea535a4b4f9f3ce5b64fe7cd7c51f6c"
    "7f93e5e49b6b92e1d0e1238256bdf78d4f7f0c1cd9a11eadf64267d0ea954c2e5b0b85c57ce1265069f2f9bb0be0082b1cc1ed8ac7c6ff5ddbb33dfed86ef9b3",
    "70205bbff3012f8017f87de0050d3de0c576c2075e90c5ff4b2ef1c776cb9fd2ccab94585fd69f10ea032f80178fd1c3d7bee8012fb613febef382b41cfdac4b"
    "fcb1fd9e1751dee06743f3be153f26847e0037801b8fd1c3d7bee80137b613febe7383b49df14597f863bbe54f223b1dcd580c64002f8017f3f7811734f48017",
    "db097fdf79415a8ebee2127f6c17a67375e38a56443d13c902c2b5357003b8317f1fb841430fb8b19df0f79d1ba4ed0ca7f43eb0dd1fda19163c0a990cf00278"
    "317f1f7841430f78b19df0811764f17fd7f66c8f3fb60fac9a2821aba611d65aba0ffaa58017f87de0050d3de0c576c2df775edc12c6ff1ddbf343fce796e90c",
    "297e8434e003f061fe3ef081861ef0613be1ef3b1f48db139f619c3b67399b31bbbe15fd09a13ef00278f1183d7ced8b1ef0623be1032fc8e2ff332ef1c7f681"
    "c6ab697e369b76457f42a80fbc005e3c460f5ffba207bcd84ef8fbce8b0961fc7fc125fed8dee725b1c11b783e545cd13889d775b139f26dd51fe007f0e3317a",
    "f8da173de0c776c2df777e90b637361defb6fc990123c377118c77032f1ede075ed0d0035e6c27fc7de7c58430fe9bb63756f6254c2903a49594a2a199c262cb"
    "76e007f063fe3ef083861ef0633be1033fc8e2bf697dad6a48b5dc9b4db12d8a63345d96c1a13e2f2decc00fe0c7fc7de0070d3de0c776c2df777e90f657bd67",
    "7bb6c71fdbf5fbfd6c33b3c3a18017c08bf9fbc00b1a7ac08bed84bfefbcb8258cffe71de33fb7589c30551ff001f880df073ed0d0033e6c27fc7de703697be2"
    "0b2ef1c7768197a4e28215cbfa13427de005f0e2317af8da173de0c576c2075e90c57fd3f5190b5e4c872e56f42784fac00be0c563f4f0b52f7ac08bed840fbc",
    "208bffa6e315535e14906e11c3e00d18af005edcbf0fbca0a107bcd84ef8c00bb2f8bf6f7bb6c71fdba7bcb856a72bfae6c0005e002fe6ef032f68e8012fb613"
    "3ef0822cfe9b9ed76a68bcacab8a8e6e786d597f42a80fbc005e3c460f5ffba207bcd84ef8c00bb2f83ba5f781edae22ad6b1af3f517d34da68017c08bf9fbc0",
    "0b1a7ac08bed84bfefbca075bef7821759d3580063f173e0067063fe3e7083861e70633be10337c8e2ff814bfcb1dda9a6066e0037e6ef033768e80137b613fe"
    "be7383b47f6ad3f332b4a5b952cbfa1342fd6df3c247c88b3f76f107db5f24b21e03c6430ed1e4c4273ff91838b1433d5a9cb81c57ab9d42b956bf2905e3fc9d",
    "70a7d70bb1087082154e9096a30f5de28fedaae59a281861b9b13c5bcabbbc3820e4c5f75cfcc176eff1626d4e3dc7df13b4eabbaf033f76aa478b1f47e7f960"
    "63acb70b47f15a4c4e8d9ae679297309fc60851fb784f1773bb755589c8ee1554e90b62bfed0c51f6cf71e2770cecc0b04ad7aecd7810b3bd5a3c58598323ccd",
    "c76f0a7e14bab8cbd44f2f03e39b28b42b98e10269ffd3a6ebbb79c110fbbc212ab2bea23f21d4f71a27fec4c51f6cf71e279672c82a23d0ffc4861e2d4ef4af"
    "63f56268a070c994d91012d2c8a8763b304ec10c276e09e3ffd38ef19f5b16bd19cc72e10f5cfcc176ef716191331b9607e0c2d3d0a3c5855ef142ca0d635ca3",
    "52686979e44f2b83443f0e5c60850b13c2f86f7a3ed274fd766e5e134d4fb6307543e9a6f811d2e6cd09cf7283747ce2072efe60bbf7b8e19c6334c7b7bfff2f"
    "bfc34feffb52afd3d6a3c511ed329d37228df3e3a4921af5c227ad7ea29982f605331ca1b5ee62a997c3aa950a99cce2e75ee507e9fcd9efb8f883eddee3c72b",
    "39352b31307ec1861e2d6e704901b52451ca47844ab4938b5e3753ad018c5f003716f7c77183036e2cee5ee70647991bd06fb55b3d5adcc81c8ff47cd1a8dc59"
    "a5bad4ed5f0ed241e518b8c10c3754c2f8bf6b7bb6c71fdbd5fb1e90795b83555efc998b3fd8ee3d5eace410f5f9b3c08bddead19a37df2e64cdd29598c8c9da",
    "715193c3478671b63c6f1e78b13efca7c20b5ae568da6b1e5ed347e5556eeceffa8bb53905fc604c8f567b63943e1644f9a6622852e4a89fedb7a2834a240afc"
    "007eccaf4dfba956475d617cc3bbfd54afe414f45331a447ad9f8a6fab85b178911487dc701cd7d50b23ad70c00d56b8f12dc2f87fcd25fed8ae2aba91d31401",
    "e9fa7c4fc292524086a9c91c6fa096a2895675b5ecd784d0af6d73e423428efca38b3fd8ee3d8e6c9273cf555af520ec3bb55b3d5a5cc95ede64d25dfe3a729e"
    "8a66eeeaea55b07fa9c4802bac708574fce33ddbb33dfed8beda4be25d7e90b643fedcc51f6cf71e3f5673087760c13c2b36f468f55b9f96c56c58ca1e5d779a",
    "cab86aaad56a4e4bf88017c08bf9f559ce015ff48aace84f08f5bdb61ef0e9ae135fca215827ce8c1ead76c575ae5091cfe2d9d2205bbbcbe86781b861c03a0e"
    "e0048eef175ce28fed3a32c2b2d056b4c533ab9cf823177fb0dd7b9cb8cf2158b7c1901eb5751b42e26cd490ae5b4954eff42e4ddd2c8dd2b06e1c38b1b86f7a",
    "eeb7550b2d1c288a63a47b9613a4fd4f7fe1e20fb67b9213cb3944799f11e0c56ef568f12224b574ff911e3732e9b2503d8df2ba1c5561fe1433bc9810c6ff99"
    "4bfcb15de5357d7e8aabce594eb590bc34c7b390c978951fa4e3df3f72f107dbbdc70fb71c837d6f59d2a3c593527e6cd4d37cb8cfc54eba91ce50c8a7b308fa",
    "a980278bfb3397f863fb6b6b270e78f2c478c2d1e6098c7bec568f164ffc4d533b19a8edd665261149e493ddab734d8279bac093c5fd175de28fedafd64e8bb1"
    "d8c596245ee509e93ac14f5dfcc1f6a7c093871ca3372f1738b25b3d5a1c394d97f267d5786a3812f2278d2e7fc70fae25d8970438b2b81373a4001c995e4f88",
    "23059a1c81feadddead1e2c85558ba280e22a7c689bf99a86a69a96c461bd01e618623a4e5e8cb2ef1c7761d19b33a693a76bbbce0dcabfc201d67ffae8b3fd8"
    "ee3d7ebc9a5334f76187736277ab478b1b41de2885c2c7e3d30c17523afebc30ea6578687f30c30d6aebcde72b96976ba475db5e7996237bbcde7c839c83f608",
    "2b7ab4b862268decf06690322fd2c7055468fbfd49a90ff3b798e1ca8430fecf5ce28fed6b6aa7954d9658e5c9131e6f77c93198bfc5921eadf5e8915a52e48c"
    "6bee94e7637ca636f68712c8f4014f58e1894a18ff2fb9c41fdb67bdee4b55b657f9e123e4c79fbaf883ed1ee4c7520ec1783a2b7ab4da1d6ab35f1e864be7dd",
    "58ef5816f4f35820d60bc33a43663841eb7bc3f267b972e214b929b67cdee5c5feeedbbe36a760df76c6f4a8cdc752cfa3b9f455dbec8a662891cede1d1b99e6"
    "25f083157e4c08e3bfed7afb96d09fb76dcf0ffecc2d829548cb7af4c6cb3d8689353fa3395e0efbe8ee568f161f3a8356af6472d95a282ce60b37814a93cfdf",
    "31b48ef0d6e1f7ddd2d17e39a5e3bee97dfb917a38fc33173d6c7f71bd5cdd86a382221b9a221d8e1449e9070b48b7eabc13ff0d6a8b82846243beab4ae87997"
    "3768fdbdfecf173fac4fefb4f4bef997cf8234f5f0c57afd373c1bf77b8d7a24ddac68f59a983ded0b5a90a17dc49f117e1ffdab43f80736fb8b44ec097c1f7d",
    "2c16504bd40da445918ae406920511e9cbe93571488fa7f63d6dbf36addfe17b1abea759d483efe9ed840f3cf96c3c212d771fb9a417b637a445249e0b664396"
    "e7ff0fabe29be2c8ff3d520f87fffb2e7ad84e81232dd59c5b974bca7d7a5b85669ad48738c129ef0ff89bd04fbf533d5adc40a1402492e90be7fd54fe2e90e3",
    "d4663bd460687ea98f901b4ff73c02fe61f2f9d27904b4b8b0c6a36b4394f46db5777ed9c50f6c776cefccbc7913eba7bd554c66c9b0ee236366a0376fe83780"
    "273bd5a336ee2b248b15a11cf6a38b1bff580fe6dac7259ea17943c093559ea80ef1dd343ddeb53ddbd303db2df7ae65b167a20cdf45beedf59b6d9b237bbb8e",
    "7a0e92956ca2d98f05eb1376ab478b1f81de383a1a5e574bc1260a088948b37e745e66681d35f063951ff4d65523d5f2f6060986a2654da3381bb95d5da33b7d"
    "cfab5c21edfffc37177fb0dd73e566ce954db28fea794ad05ed9ad1eadf9e9354da885cc1a6ad6f36143aa9a5ac69496bf2f8137ebc3df57defcaa4b7a60fba2",
    "c24a742dcf5f831b6679b3f9789bc7cacd0a6f5e977d74d75f036f76ab478b37c18c1cecd4a55eeb9c2bb54bd95052bac82b3ee00dabbc212d573fe7921ed8be"
    "a8b00a990c8724c9aab254737650b857f9423adef2b72efe60bbe7cac90a5f56b20bce8365488f567f59d1e0bb03e15ab88ea1d3f3981ee7d205a102e3f7ccf2",
    "64e210df4dd3e3975cd203db7105a50cd2fcdd6cbfa179dfcb6c3b229f77b942ba5fd43fb8f883ed9e2b2fab5c599b6d34c763a0bdb25b3d5a7ca9f84797cde0"
    "1977c49f2692f960e7aa972a94613c1ff8e2901e1b9fb3f14a4535eb6bc17861962f7fefe20fb67baebc38f1e521db60fc85213d5a7c396fe764fe2a6e4a5ce0",
    "2837aab7d227c9b2ccd03a48e0cb76f9f2cc253db07d5151718a6476e535dfc25ee50be9b8cb3fb9f883ed9e2b2f2b7c71cc36eafb51016776ab478b3342b659"
    "192bc742ece8acd3ebe6838172fd08413b0638e3901ebfe2921ed8beb6c25a6aca7895337b7b4ec76b38b3dc9401beb0a1476b5cffd2a80989e8b97c64a8d171",
    "30de2ff753c9e57e72e0cbfaf09f2a5fa8edc33cafa8d2a22cd6794368e313a0bcca15d271fd1fbaf883ed9e2b272b5cb165d752b9b975883ff0e469e8d1fabb"
    "37d4443c1812c4f3aad62f9a916063ac8c0486f6cf059e6c775ef2c72ee981ed8b0a6a3ee3c8e970419f77f942da3ff6cf2efe60bbe7cacd0a5f9cb30f779001",
    "6fd8d0a3c59bd4d59d1938c925a58a74972fdd04cee5be5103de30cb9b89437c77348fecbec25a39c2cee75dcec03c32e76c837964ece8d1ea1feb8e4e4a5c35"
    "7beacfd4c799a6111706274adc077c61952fa4e5ea0397f4c076cb3dced40da59be247c8aaa614d5dbeb2a49fbc7beefe20fb67bae9cdcef0f63cf2e5847c992",
    "1e2d9e24f9d3ce15e202a1cbb891ace48eb5d4f115aca3649727aa437cb7bdcf98d0464227d18c2b5203357cdee5c89eef33b6924dd01e61478f567f57550ed5"
    "5af5ae68d4447d90ea0c4f7823e0877593ccf283e279b7f34e93a2389e6e1832fbcef5799723a4ed911fb8f883ed9e2b27f7ed9157b30b78c28e1e2d9e6443a3",
    "8b30e2d049a314bb4a078ff399233306e327c01387f4f8b24b7a60bbe55e223beb275982898f5d9efc8d8b3fd8eeb97272cf135b76513e970578b25b3d5a3c69"
    "2786d9b07ce7efc5c6d9e4b111d2a5c1a80beb5580270ee9f11597f4c076cbbda2e53b6a24b2b3ea2921374401e9ccf2e453177fb0dd73e5e49e27ebb20bd6a7",
    "b0a2478b27fa55406ce997c961e03897ce1d19c3fca8dd877df599e509adf112cb3d5c31a16959f62a47f67cbc64259ba07f8b1d3d5afddabd52ae122c70c9cb"
    "7622d6ea66c481ace6ab3ee007f0637d7a38a5ff81ed6eb9375d2c17992e964bc84dc5abfc206d877ccfc51f6cf75cf9b8e7c74a36c13e2c8ce9513b6f38d939",
    "ad996638d00fd6457d9cce374ee21af46b31cb9189437cb7bdce64da51827a269205944272cb682fb65687fd8ae797e7cacb43ff9663b6413b851d3d5a7cb9ba"
    "b92b97a2dd48779c45464cbbd6b8f3e68803beb0ca17d276cafbb6677b7a60fbfd44a1c5ca45aff284b49febaf5dfcc176cf950fdb7c2ebc5211f8c1861ead7e",
    "ae316f48a5b2163ebf307b3ddd10cf8a032ee2037eb0ca0feaf382e388374c0d4de79b5a9f259ee508cc0b7e35bba03dc28e1eadf60857d5fdda4de4fa941bd6"
    "abed6ba9701150d3302f98599ed03acf7e5d05b5667b15cff2e51961b979e2e7d96f927d709e0a437ab4785388c6fdb5fe597edcbd532e46a893ca84d5189ca7",
    "c22c6f28ce1b5e5928b7a8a93ccb179837bc2ebba03f8c153d5afd168562ed7814290dab5c372c8df28156a8d15ede170378b23efc7de5c9a6e7d5cf37f0901f"
    "6a288fb757f6fcbcfa75d905e7d533a4478b27b5f3cb7ef5a6df8a5f8b52b95fbb92b574a2e0039eb0ca13d521be9ba6c77bb6677b7a60fb6a05e55d8e908ecf",
    "ff958b3fd8eeb9f2b18e23702e0a337ab4fab7065781bbeb5ab414e2c7e966395bce044e0c1ef6ed62961fb4be4bfabc24367803e1d9a85e9f37bce7e3f36bb3"
    "0bc6e7d9d1a3c5936287bf190f95e8b05c1eb68e32e958f4d82c014f98e509697bc429fd0f6c775c41cd565d87b516ab1c79e2eb1aedd9447d5d23f46bed568f",
    "16476e02e3561b0dbb47e58b684036235721bd52b9008eb0ca11d272f5914b7a60fbacc3242c37ca96537145bbe4a5e6ec5bd7ab3c215dcff8772efe60bbe7ca"
    "c952ffd69aec82f5f28ce9d1ea97a88f9551aead4bd5c66545bdaae58f32853c8cbfb3cb9589437cdf547d7efb487fec97933ff8a23f6fcb5bc5618e0f2703cc",
    "db62458f567be45448162b4239ec471737feb11eccb58f4b3cecb302dc70488fafbaa407b63b7263617f53dca0b7bfa3c78ac39a9fd11c07f9c6a73f065eec50"
    "8f162f3a8356af6472d95a282ce60b37814a93cfdf31d47f455a3f7cc721fc039bddd3bc882b1a679bb73971883770e3f57ac08dd7a70b70e3cdea0137b6133e",
    "706337dc807eaad7eb413fd5faf4817e2a3a7ad04f4516feff0305eebda9", "" };

  c5_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c5_data[0], 142048U, &c5_nameCaptureInfo);
  return c5_nameCaptureInfo;
}

static void c5_eML_blk_kernel(SFc5_LaneDetectionInstanceStruct *chartInstance,
  real32_T c5_b_In[1229760], real_T c5_b_bboxes_data[], int32_T c5_bboxes_size[2],
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
  c5_eML_blk_kernel_kernel2<<<dim3(4U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_aux2);
  c5_eML_blk_kernel_kernel3<<<dim3(4U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_aux1, *chartInstance->c5_gpu_rowWeights,
     *chartInstance->c5_gpu_ipRowIndices);
  c5_eML_blk_kernel_kernel4<<<dim3(7U, 1U, 1U), dim3(512U, 1U, 1U)>>>
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
  for (c5_b_k = 0; c5_b_k < 15; c5_b_k++) {
    c5_eML_blk_kernel_kernel8<<<dim3(1U, 1U, 1U), dim3(224U, 1U, 1U)>>>
      (*chartInstance->c5_gpu_colWeights, (c5_b_k + 1) * 224,
       *chartInstance->c5_gpu_colWeightsTotal);
  }

  cudaMemcpy(chartInstance->c5_gpu_In, &c5_b_In[0], 4919040UL,
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
    (*chartInstance->c5_gpu_out, *chartInstance->c5_gpu_outVal);
  c5_coder_reduce0<<<dim3(294U, 1U, 1U), dim3(512U, 1U, 1U)>>>
    (*chartInstance->c5_gpu_out, *chartInstance->c5_gpu_outVal);
  cudaMemcpy(&c5_outVal[0], chartInstance->c5_gpu_outVal, 8UL,
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
         1, *chartInstance->c5_e_gpu_bboxPred_data);
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

      cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_bboxPred_size[0],
                 8UL, cudaMemcpyHostToDevice);
      c5_bboxPred_size_dirtyOnCpu = false;
      c5_eML_blk_kernel_kernel32<<<c5_r_grid, c5_r_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
         *chartInstance->c5_c_gpu_bboxPred_size, c5_i25 - 1,
         *chartInstance->c5_e_gpu_bboxPred_data);
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
        cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel33<<<c5_s_grid, c5_s_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
         *chartInstance->c5_c_gpu_bboxPred_size, c5_i26 - 1,
         *chartInstance->c5_e_gpu_bboxPred_data);
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
        cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel34<<<c5_t_grid, c5_t_block>>>
        (*chartInstance->c5_gpu_bboxesX1Y1X2Y2_data,
         *chartInstance->c5_gpu_bboxesX1Y1X2Y2_size,
         *chartInstance->c5_c_gpu_bboxPred_size, c5_i27 - 1,
         *chartInstance->c5_e_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_nx = c5_bboxPred_size[0] << 2;
    c5_u_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c5_nx - 1)
      + 1L), &c5_u_grid, &c5_u_block, 1024U, 65535U);
    if (c5_u_validLaunchParams) {
      c5_eML_blk_kernel_kernel35<<<c5_u_grid, c5_u_block>>>(c5_nx,
        *chartInstance->c5_e_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_i32 = c5_bboxPred_size[0] - 1;
    c5_b_bboxPred_size[0] = c5_bboxPred_size[0];
    c5_v_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i32 + 1L),
      &c5_v_grid, &c5_v_block, 1024U, 65535U);
    if (c5_v_validLaunchParams) {
      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel36<<<c5_v_grid, c5_v_block>>>
        (*chartInstance->c5_e_gpu_bboxPred_data,
         *chartInstance->c5_c_gpu_bboxPred_size, c5_i32,
         *chartInstance->c5_d_gpu_bboxPred_data);
    }

    c5_w_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_b_bboxPred_size[0] - 1) + 1L), &c5_w_grid, &c5_w_block, 1024U, 65535U);
    if (c5_w_validLaunchParams) {
      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_b_gpu_bboxPred_size, &c5_b_bboxPred_size[0],
                 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel37<<<c5_w_grid, c5_w_block>>>
        (*chartInstance->c5_d_gpu_bboxPred_data,
         *chartInstance->c5_c_gpu_bboxPred_size,
         *chartInstance->c5_b_gpu_bboxPred_size,
         *chartInstance->c5_e_gpu_bboxPred_data);
      c5_b_bboxPred_data_dirtyOnGpu = true;
    }

    c5_i35 = c5_bboxPred_size[0] - 1;
    c5_c_bboxPred_size[0] = c5_bboxPred_size[0];
    c5_x_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c5_i35 + 1L),
      &c5_x_grid, &c5_x_block, 1024U, 65535U);
    if (c5_x_validLaunchParams) {
      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      c5_eML_blk_kernel_kernel38<<<c5_x_grid, c5_x_block>>>
        (*chartInstance->c5_e_gpu_bboxPred_data,
         *chartInstance->c5_c_gpu_bboxPred_size, c5_i35,
         *chartInstance->c5_gpu_bboxPred_data);
    }

    c5_y_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_c_bboxPred_size[0] - 1) + 1L), &c5_y_grid, &c5_y_block, 1024U, 65535U);
    if (c5_y_validLaunchParams) {
      if (c5_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
        c5_bboxPred_size_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_e_gpu_bboxPred_size, &c5_c_bboxPred_size[0],
                 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel39<<<c5_y_grid, c5_y_block>>>
        (*chartInstance->c5_gpu_bboxPred_data,
         *chartInstance->c5_c_gpu_bboxPred_size,
         *chartInstance->c5_e_gpu_bboxPred_size,
         *chartInstance->c5_e_gpu_bboxPred_data);
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
        cudaMemcpy(&c5_bboxPred_data[0], chartInstance->c5_e_gpu_bboxPred_data,
                   25088UL, cudaMemcpyDeviceToHost);
        c5_b_bboxPred_data_dirtyOnGpu = false;
      }

      if ((c5_bboxPred_data[c5_d_i + c5_bboxPred_size[0] * 3] >= 1.0) &&
          (c5_bboxPred_data[c5_d_i + (c5_bboxPred_size[0] << 1)] >= 1.0) &&
          (c5_bboxPred_data[c5_d_i + c5_bboxPred_size[0] * 3] <= 480.0) &&
          (c5_bboxPred_data[c5_d_i + (c5_bboxPred_size[0] << 1)] <= 854.0)) {
        c5_count++;
        if (c5_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_size, &c5_bboxPred_size[0],
                     8UL, cudaMemcpyHostToDevice);
          c5_bboxPred_size_dirtyOnCpu = false;
        }

        if (c5_b_bboxPred_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_bboxPred_size, &c5_d_bboxPred_size[0],
                     8UL, cudaMemcpyHostToDevice);
          c5_b_bboxPred_size_dirtyOnCpu = false;
        }

        c5_eML_blk_kernel_kernel40<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
          (*chartInstance->c5_e_gpu_bboxPred_data,
           *chartInstance->c5_c_gpu_bboxPred_size, c5_d_i,
           *chartInstance->c5_gpu_bboxPred_size, (int32_T)c5_count - 1,
           *chartInstance->c5_c_gpu_bboxPred_data);
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
         *chartInstance->c5_gpu_idx_data, *chartInstance->c5_c_gpu_bboxPred_data);
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
        cudaMemcpy(chartInstance->c5_gpu_n, &c5_n, 4UL, cudaMemcpyHostToDevice);
        c5_eML_blk_kernel_kernel44<<<c5_cb_grid, c5_cb_block>>>
          (*chartInstance->c5_gpu_b_size, *chartInstance->c5_gpu_b_data, c5_i3,
           chartInstance->c5_gpu_n);
        c5_n_dirtyOnCpu = false;
      }

      if (c5_b_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_bboxPred_size, &c5_d_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
      }

      if (c5_n_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_gpu_n, &c5_n, 4UL, cudaMemcpyHostToDevice);
      }

      c5_eML_blk_kernel_kernel45<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (chartInstance->c5_gpu_n, *chartInstance->c5_gpu_bboxPred_size,
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
                         chartInstance->c5_c_gpu_bboxPred_data, 25088UL,
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
        cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_data, &c5_b_bboxPred_data[0],
                   25088UL, cudaMemcpyHostToDevice);
        c5_bboxPred_data_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_e_bboxPred_size[0],
                 8UL, cudaMemcpyHostToDevice);
      c5_c_bboxPred_size_dirtyOnCpu = false;
      c5_eML_blk_kernel_kernel47<<<c5_db_grid, c5_db_block>>>
        (*chartInstance->c5_c_gpu_bboxPred_data,
         *chartInstance->c5_gpu_bboxPred_size,
         *chartInstance->c5_d_gpu_bboxPred_size, c5_i4,
         *chartInstance->c5_b_gpu_bboxPred_data);
    }

    c5_d_bboxPred_size[0] = c5_e_bboxPred_size[0];
    c5_d_bboxPred_size[1] = 4;
    c5_b_bboxPred_size_dirtyOnCpu = true;
    c5_eb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c5_e_bboxPred_size[0] * 4 - 1) + 1L), &c5_eb_grid, &c5_eb_block, 1024U,
      65535U);
    if (c5_eb_validLaunchParams) {
      if (c5_c_bboxPred_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c5_d_gpu_bboxPred_size, &c5_e_bboxPred_size[0],
                   8UL, cudaMemcpyHostToDevice);
      }

      c5_eML_blk_kernel_kernel48<<<c5_eb_grid, c5_eb_block>>>
        (*chartInstance->c5_b_gpu_bboxPred_data,
         *chartInstance->c5_d_gpu_bboxPred_size,
         *chartInstance->c5_c_gpu_bboxPred_data);
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
      cudaMemcpy(chartInstance->c5_b_gpu_n, &c5_b_n, 4UL, cudaMemcpyHostToDevice);
      c5_eML_blk_kernel_kernel51<<<c5_hb_grid, c5_hb_block>>>
        (*chartInstance->c5_gpu_b_size, *chartInstance->c5_gpu_b_data, c5_i6,
         chartInstance->c5_b_gpu_n);
      c5_n_dirtyOnGpu = true;
    }

    if (c5_n_dirtyOnGpu) {
      cudaMemcpy(&c5_b_n, chartInstance->c5_b_gpu_n, 4UL, cudaMemcpyDeviceToHost);
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
           *chartInstance->c5_gpu_scores_data);
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
          cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_data, &c5_b_bboxPred_data
                     [0], 25088UL, cudaMemcpyHostToDevice);
          c5_bboxPred_data_dirtyOnCpu = false;
        }

        cudaMemcpy(chartInstance->c5_gpu_x1_size, &c5_x1_size[0], 4UL,
                   cudaMemcpyHostToDevice);
        c5_x1_size_dirtyOnCpu = false;
        c5_eML_blk_kernel_kernel57<<<c5_mb_grid, c5_mb_block>>>
          (*chartInstance->c5_c_gpu_bboxPred_data,
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
          cudaMemcpy(chartInstance->c5_c_gpu_bboxPred_data, &c5_b_bboxPred_data
                     [0], 25088UL, cudaMemcpyHostToDevice);
        }

        if (c5_iv1_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c5_gpu_iv1_data, &c5_iv1_data[0], 1568UL,
                     cudaMemcpyHostToDevice);
        }

        cudaMemcpy(chartInstance->c5_gpu_iv1_size, &c5_iv1_size[0], 4UL,
                   cudaMemcpyHostToDevice);
        c5_eML_blk_kernel_kernel65<<<c5_wb_grid, c5_wb_block>>>
          (*chartInstance->c5_c_gpu_bboxPred_data,
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
      cudaMemcpy(chartInstance->c5_gpu_scores_data, &c5_c_scores_data[0], 3136UL,
                 cudaMemcpyHostToDevice);
    }

    cudaMemcpy(chartInstance->c5_gpu_scores_size, &c5_b_scores_size[0], 4UL,
               cudaMemcpyHostToDevice);
    c5_eML_blk_kernel_kernel68<<<c5_e_grid, c5_e_block>>>
      (*chartInstance->c5_gpu_scores_data, *chartInstance->c5_gpu_scores_size,
       chartInstance->c5_b_gpu_scores_data);
    c5_scores_data_dirtyOnGpu = true;
  }

  if (c5_bboxes_data_dirtyOnGpu) {
    cudaMemcpy(&c5_b_bboxes_data[0], chartInstance->c5_gpu_bboxes_data,
               (uint32_T)(c5_bboxes_size[0] * 4) * sizeof(real_T),
               cudaMemcpyDeviceToHost);
  }

  if (c5_scores_data_dirtyOnGpu) {
    cudaMemcpy(&c5_b_scores_data[0], chartInstance->c5_b_gpu_scores_data,
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
  (int16_T c5_aux2[1708])
{
  int32_T c5_i;
  c5_i = (int32_T)mwGetGlobalThreadIndex();
  if (c5_i < 1708) {
    if (c5_i + 1 <= 854) {
      c5_aux2[c5_i] = (int16_T)(c5_i + 1);
    } else {
      c5_aux2[c5_i] = (int16_T)(1708 - c5_i);
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
  int16_T c5_aux2[1708], real_T c5_colWeights[3584], int16_T c5_ipColIndices
  [3584])
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
  c5_k = (int32_T)(c5_threadId % 16UL);
  c5_colIdx = (int32_T)((c5_threadId - (uint64_T)c5_k) / 16UL);
  if ((c5_colIdx < 224) && (c5_k < 16)) {
    c5_ipColIdx = ((real_T)c5_colIdx + 1.0) / 0.26229508196721313 + -1.40625;
    c5_colIndices = (int32_T)floor(c5_ipColIdx - 7.625);
    c5_absx = fabs(0.26229508196721313 * (c5_ipColIdx - ((real_T)(c5_colIndices
      + c5_k) + 1.0)));
    c5_absx2 = c5_absx * c5_absx;
    c5_absx3 = pow(c5_absx, 3.0);
    c5_oldIdx = (c5_colIndices + c5_k) + 1;
    if (c5_oldIdx - 1 == 0) {
      c5_l = 0;
    } else {
      c5_l = (int32_T)fmod((real_T)c5_oldIdx - 1.0, 1708.0);
      if ((c5_l != 0) && (c5_oldIdx - 1 < 0)) {
        c5_l += 1708;
      }
    }

    c5_ipColIndices[c5_colIdx + 224 * c5_k] = c5_aux2[c5_l];
    c5_colWeights[c5_colIdx + 224 * c5_k] = 0.26229508196721313 * (((1.5 *
      c5_absx3 - 2.5 * c5_absx2) + 1.0) * (real_T)(c5_absx <= 1.0) + (((-0.5 *
      c5_absx3 + 2.5 * c5_absx2) - 4.0 * c5_absx) + 2.0) * (real_T)((1.0 <
      c5_absx) && (c5_absx <= 2.0)));
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
  real_T c5_colWeights[3584], real_T c5_colWeightsTotal[224])
{
  int32_T c5_j;
  c5_j = (int32_T)mwGetGlobalThreadIndex();
  if (c5_j < 224) {
    c5_colWeightsTotal[c5_j] = c5_colWeights[c5_j];
  }
}

static __global__ __launch_bounds__(224, 1) void c5_eML_blk_kernel_kernel8(const
  real_T c5_colWeights[3584], const int32_T c5_xoffset, real_T
  c5_colWeightsTotal[224])
{
  int32_T c5_j;
  c5_j = (int32_T)mwGetGlobalThreadIndex();
  if (c5_j < 224) {
    c5_colWeightsTotal[c5_j] += c5_colWeights[c5_xoffset + c5_j];
  }
}

static __global__ __launch_bounds__(512, 1) void c5_eML_blk_kernel_kernel9(const
  real_T c5_colWeightsTotal[224], const real_T c5_colWeights[3584], const
  real32_T c5_b_In[1229760], const int16_T c5_ipColIndices[3584], real32_T
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
    for (c5_l = 0; c5_l < 16; c5_l++) {
      c5_sumVal += (real_T)c5_b_In[(c5_rowIdx + 480 * ((int32_T)
        c5_ipColIndices[c5_colIdx + 224 * c5_l] - 1)) + 409920 * c5_dimIdx] *
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
    c5_bboxPred_data[c5_i28] = ((c5_bboxesX1Y1X2Y2_data[c5_i28] - 0.5) * 3.8125
      + -1.40625) + 0.5;
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
       * 3.8125 + -1.40625) - 0.5;
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
  chartInstance->c5_In = (real32_T (*)[1229760])ssGetInputPortSignal_wrapper
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
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(3571303581U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(3804992752U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(2354264699U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(514449129U);
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
  return "sLBlpmjqljmjH7g47RTOyuD";
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
    "eNrtXE9vG0UUd9JSUQmqHpCQKiR6AwkJJ7ZDKw7Qxn9US0kTxWkRJ2tmduydeHd2szNrJypCon9",
    "OfAQucCucOHDgyIEDH4Er34CPwJu142zWu3Y3KbGfVUuOs+s3b3/v77yZfevCSnO7AK8b8H7JC4",
    "Vr8Pk2vFcLw9dbo+OV2Ht4/mrho9HxbzBIhu4uCYirClNfkrh8jyvPCbXwZFN2vFQyITs84JIBr",
    "e8FOoubEm7oCNlrhJIZfuorWzC7ZXuhY23CWGLtSOcYuPmh3gU+NRFwphucW9oOvLBrNxzSHSMO",
    "9KBqc9ZToTtNBMV1K/QNLLUdOlr4Dq8fcdaUShNArE6xtTTRvKqPMsU0kqrWCaHn+o4gMlVam6g",
    "W90HBmj/yLfi7E2oQKknGbBLoTW6TPldbohfx9CRP8hQKvqBCEu0Fgjh116magZPYdh3As+1Z3J",
    "miEMC2GXDS8z0hdbb9Ww2QtC4JdXiN07Cbza3FD0Nj/MeCD3iQqbdO1evzgHT5jsy8aKSQ+lFkr",
    "bGXTJJp4fLHJLjPwH6KW5neC56jWgTsxPdhRBYZj4Rsqv1A9EG9mdxCt2k8c1bIhO7Q2GoWWcSt",
    "3ufTrDDm1mCyShxHZZLte/4W73Mn4lojmkwnG3JNp1NKWPseKNi4d3Y0hFKA4UdkVU9aItVc/QR",
    "BlHceQmI5S8lCpT23Cs5b29qa/HqSrCk1DzqE8bQsEBChOOgsUm82N0soY3sgBFQ6gpdGPPSQWV",
    "QF1QllbeAFPdDJlCRyKoKxaCahq7pgS4iERwqCZhqZseUsOkaYzS2TYITDtyFsgDZFJ8qktvsQd",
    "32hj2tcsUD4KVYNIeogDdWNQx37/JHsSW8gG4HntkY5fmgFyAyQw12wwX4UY5IBK6E0pAtxenmL",
    "c3BKEkghu5uQ5oLjBoBMtZiZ99YKp/PeO68w752MS35+HOOzksKnEPtMXvf66vTrrsJ/K6Nx92L",
    "j3k1c52pinKG7Ce+fP/n62z+//+fu5z/9UP+F/PHNRa5/dCVfnXBjdPzBSUIeB1h/wq8N7YMYrq",
    "sp/N+P8b85OlZbm47vHhw6B+7Bgzvdyp29/Z3jsBbx+2t1Ot4rCbwn52+bmQG8MfLjgDWtUQFjj",
    "kk4nNYN/7sxvNdm6OP66Pzw9e+XFxv/3r3k+DR9XUvoyxxT6h2ZiIj77/zkuH1uORTzgtcnxySO",
    "fOOH19+dIcethBy3onqmTUyW5G220d4iEmokzaNyZTLP5I3bN+PejFvkcSvnnIfPO271gvP+ZY2",
    "7qHx565FFo1+bkkcLCfqbCyzHRevE/5v+70K+eu7D0fEX46Vd1RaOlVLlj76GWryT9u2S+Km/N5",
    "3eT9D/PqM++C7h1+a4aHsuL9ouc4r3a7Do0YHnFJXjBwfFtuqYjzNFA5zkTqdoFpBFKF2LTMr2s",
    "ed4/dIeVw+53lh7zG3BzNZVtJ2xdrbmWBsRwxq7X2rTT6mQaPGvtwc58b9I4H8xP/wOUcoIMZYB",
    "K36aA//zBP7nc8APi4pKp00DIpldYqcuhBQ/xYifI9f/GD9Fjx+3/xDk8YvdfwhO/7GQ508Luf9",
    "byP2HIZ+/GHL/Z8vi/xS9/lHip8jjlyKPX4q8/qG447dEcefPOH6U+ifLov9E/P44A7+XwO+9Hv",
    "w6IEJyy9CBBBOIO8ycWB+5CiY9M+R5kpwzTp8l8D+bc5yux3bKF93PS/j8vEwy5tNfZ+B/ksD/Z",
    "A74WeJOBMp5KGcdvHjxiVz/2NeBFHkdlnMd9TSB/+kc8EfPPrTX216no7g+vRGKMu/jrOPLZFnu",
    "IyLdxyHI90E6GflzVn3pJvC7l1JfsjM9I5ji9Lz7xYvQ73KS5xUjDk/0uyyun1Ry+cmi9UUN0Oi",
    "5nKt/aFHikS7LvI9z/65snXP/bn5+vnHGzxd5fkQYjyXk98PK1vLsp1M8++mD8T7jIs+PFEXe2E",
    "BZV9Nl6YNBMw+uY/STUla/Ap79fYpxf7nMlmV/hGCJz8qZ+u/lDJyHCZyHl4LTIppMbBu/kl57C",
    "by9S6o3Pst5/3V+OAfY88QA+fMkFHk/KMr1C/J+0BLyfuLy0uifIsfP0OPH2AdSyerTQoK/vDz4",
    "cdY/BPf8W0Z+H7yMvB+9jPx5gPjzkAPUfVwUef1Gkccvfv1jv/9FsfW3xH/PJe/v0Szg8zwUeT8",
    "A7vzD8t0/9RP4/cvbfx23c6H0E4bcz9Gvc1GusyrI+6YqBGGfRinWp4FQz7ifDyd4+goQ9q1VyL",
    "L8jkOsbvwPObOpPQ==",
    ""
  };

  static char newstr [2269] = "";
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
  ssSetChecksum0(S,(2119773097U));
  ssSetChecksum1(S,(954305476U));
  ssSetChecksum2(S,(1167827258U));
  ssSetChecksum3(S,(2092982698U));
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
