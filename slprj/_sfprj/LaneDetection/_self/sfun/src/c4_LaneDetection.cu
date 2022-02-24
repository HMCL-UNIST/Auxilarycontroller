/* Include files */

#include "LaneDetection_sfun.h"
#include "c4_LaneDetection.h"
#include "MWCudaDimUtility.hpp"
#include "MWLaunchParametersUtilities.hpp"
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
static emlrtMCInfo c4_emlrtMCI = { 47, /* lineNo */
  5,                                   /* colNo */
  "repmat",                            /* fName */
  "/usr/local/MATLAB/R2020b/toolbox/eml/lib/matlab/elmat/repmat.m"/* pName */
};

static __device__ real32_T c4_gpu_In[921600];
static __device__ real32_T c4_gpu_In_1[921600];

/* Function Declarations */
static void initialize_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static void initialize_params_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct *
  chartInstance);
static void enable_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static void disable_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static void c4_do_animation_call_c4_LaneDetection
  (SFc4_LaneDetectionInstanceStruct *chartInstance);
static void ext_mode_exec_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static const mxArray *get_sim_state_c4_LaneDetection
  (SFc4_LaneDetectionInstanceStruct *chartInstance);
static void set_sim_state_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c4_st);
static void sf_gateway_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_start_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_terminate_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static void mdl_setup_runtime_resources_c4_LaneDetection
  (SFc4_LaneDetectionInstanceStruct *chartInstance);
static void mdl_cleanup_runtime_resources_c4_LaneDetection
  (SFc4_LaneDetectionInstanceStruct *chartInstance);
static void initSimStructsc4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static void c4_emlrt_marshallIn(SFc4_LaneDetectionInstanceStruct *chartInstance,
  const mxArray *c4_c_In, const char_T *c4_identifier, real32_T c4_y[921600]);
static void c4_b_emlrt_marshallIn(SFc4_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real32_T c4_y[921600]);
static uint8_T c4_c_emlrt_marshallIn(SFc4_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_LaneDetection, const char_T
  *c4_identifier);
static uint8_T c4_d_emlrt_marshallIn(SFc4_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId);
static void c4_eML_blk_kernel(SFc4_LaneDetectionInstanceStruct *chartInstance,
  boolean_T c4_b_laneFound, real32_T c4_b_ltPts[56], real32_T c4_b_rtPts[56],
  real_T c4_b_bboxes_data[], int32_T c4_bboxes_size[2], real32_T
  c4_b_scores_data[], int32_T c4_scores_size[2], real32_T c4_c_In[921600]);
static __global__ void c4_sf_gateway_c4_LaneDetection_kernel1();
static __global__ void c4_eML_blk_kernel_kernel2(const real_T c4_b_bboxes_data[],
  const int32_T c4_bboxes_size[2], int32_T c4_position_data[80]);
static __global__ void c4_eML_blk_kernel_kernel3(const int8_T c4_color_data[60],
  const int32_T c4_color_size[2], int8_T c4_b_color_data[60]);
static __global__ void c4_eML_blk_kernel_kernel4(real32_T c4_c_In[921600],
  real32_T c4_b_I[921600]);
static __global__ void c4_eML_blk_kernel_kernel5(uint8_T c4_pixCount[640]);
static __global__ void c4_eML_blk_kernel_kernel6(const int32_T c4_position_data
  [80], const int32_T c4_position_size[2], int32_T c4_positionOut_data[112]);
static __global__ void c4_eML_blk_kernel_kernel7(const int8_T c4_color_data[60],
  const int32_T c4_color_size[2], real32_T c4_b_color_data[60]);
static __global__ void c4_eML_blk_kernel_kernel8(const int32_T c4_position_data
  [80], const int32_T c4_position_size[2], const int32_T
  c4_textLocAndWidth_size[2], const int32_T c4_i25, int32_T
  c4_textLocAndWidth_data[80]);
static __global__ void c4_eML_blk_kernel_kernel9(const int32_T
  c4_textLocAndWidth_data[80], const int32_T c4_textLocAndWidth_size[2], const
  int32_T c4_i28, int32_T c4_b_textLocAndWidth_data[20]);
static __global__ void c4_eML_blk_kernel_kernel10(const int32_T
  c4_textLocAndWidth_data[20], const int32_T c4_textLocAndWidth_size[2], const
  int32_T c4_b_textLocAndWidth_size[1], int32_T c4_b_textLocAndWidth_data[80]);
static __global__ void c4_eML_blk_kernel_kernel11(const int32_T
  c4_textLocAndWidth_data[80], const int32_T c4_textLocAndWidth_size[2], const
  int32_T c4_textPosition_size[2], const int32_T c4_i34, int32_T
  c4_textPosition_data[40]);
static __global__ void c4_eML_blk_kernel_kernel12(const int8_T c4_color_data[60],
  const int32_T c4_color_size[2], int8_T c4_textColor_data[60]);
static __global__ void c4_eML_blk_kernel_kernel13(const int32_T
  c4_textLocAndWidth_data[80], const int32_T c4_textLocAndWidth_size[2], const
  int32_T c4_iv[2], int32_T c4_shapeWidth_data[20]);
static __global__ void c4_eML_blk_kernel_kernel14(const int32_T
  c4_textLocAndWidth_data[80], int32_T c4_shapeWidth_data[20]);
static __global__ void c4_eML_blk_kernel_kernel15(const int32_T
  c4_textLocAndWidth_data[80], const int32_T c4_textLocAndWidth_size[2], const
  int32_T c4_iv1[2], int32_T c4_shapeHeight_data[20]);
static __global__ void c4_eML_blk_kernel_kernel16(const int32_T
  c4_textLocAndWidth_data[80], int32_T c4_shapeHeight_data[20]);
static __global__ void c4_eML_blk_kernel_kernel17(char_T c4_str1[30]);
static __global__ void c4_eML_blk_kernel_kernel18(const char_T c4_cv8[6], char_T
  c4_cv7[6]);
static __global__ void c4_eML_blk_kernel_kernel19(const char_T c4_str1[30],
  const int32_T c4_i1, uint8_T c4_thisTextU16_data[29]);
static __global__ void c4_eML_blk_kernel_kernel20(const uint8_T
  c4_thisTextU16_data[29], const int32_T c4_thisTextU16_size[2], boolean_T
  c4_isNewLineChar_data[29]);
static __global__ void c4_eML_blk_kernel_kernel21(const int8_T c4_ii_data[29],
  const int32_T c4_ii_size[2], int8_T c4_idxNewlineChar_data[29]);
static __global__ void c4_eML_blk_kernel_kernel22(const uint8_T
  c4_thisTextU16_data[29], const int32_T c4_i3, uint16_T
  c4_thisCharcodes_1b_data[29]);
static __global__ void c4_eML_blk_kernel_kernel23(const int8_T c4_iv1[261],
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], const
  int32_T c4_thisCharcodes_1b_size[2], int8_T c4_x_data[29]);
static __global__ void c4_eML_blk_kernel_kernel24(const int8_T c4_iv1[261],
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], real_T
  *c4_lenFirstSegment);
static __global__ void c4_eML_blk_kernel_kernel25(const int32_T c4_x_size[2],
  const int8_T c4_x_data[29], int32_T c4_vlen, real_T *c4_lenFirstSegment);
static __device__ real_T c4_threadGroupReduction(real_T c4_val, uint32_T c4_lane,
  uint32_T c4_mask);
static __device__ real_T c4_shflDown2(real_T c4_in1, uint32_T c4_offset,
  uint32_T c4_mask);
static __device__ real_T c4_workGroupReduction(real_T c4_val, uint32_T c4_mask,
  uint32_T c4_numActiveWarps);
static __device__ real_T c4_atomicOpreal_T(real_T *c4_address, real_T c4_value);
static __global__ void c4_eML_blk_kernel_kernel26(const uint16_T c4_uv[256],
  const uint16_T c4_thisCharcodes_1b_data[29], const int32_T
  c4_thisCharcodes_1b_size[2], boolean_T c4_isNewLineChar_data[29]);
static __global__ void c4_eML_blk_kernel_kernel27(const boolean_T
  c4_isNewLineChar_data[29], int32_T *c4_nz);
static __global__ void c4_eML_blk_kernel_kernel28(const int32_T
  c4_isNewLineChar_size[2], const boolean_T c4_isNewLineChar_data[29], int32_T
  c4_vlen, int32_T *c4_nz);
static __device__ int32_T c4_b_threadGroupReduction(int32_T c4_val, uint32_T
  c4_lane, uint32_T c4_mask);
static __device__ int32_T c4_shflDown1(int32_T c4_in1, uint32_T c4_offset,
  uint32_T c4_mask);
static __device__ int32_T c4_b_workGroupReduction(int32_T c4_val, uint32_T
  c4_mask, uint32_T c4_numActiveWarps);
static __global__ void c4_eML_blk_kernel_kernel29(const uint8_T
  c4_thisTextU16_data[29], const int32_T c4_i6, const int32_T c4_i8, uint16_T
  c4_thisCharcodes_1b_data[29]);
static __global__ void c4_eML_blk_kernel_kernel30(const int8_T c4_iv1[261],
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], const
  int32_T c4_thisCharcodes_1b_size[2], int8_T c4_x_data[29]);
static __global__ void c4_eML_blk_kernel_kernel31(const int8_T c4_iv1[261],
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], real_T
  *c4_lenThisSegment);
static __global__ void c4_eML_blk_kernel_kernel32(const int32_T c4_x_size[2],
  const int8_T c4_x_data[29], int32_T c4_vlen, real_T *c4_lenThisSegment);
static __global__ void c4_eML_blk_kernel_kernel33(const uint16_T c4_uv[256],
  const uint16_T c4_thisCharcodes_1b_data[29], const int32_T
  c4_thisCharcodes_1b_size[2], boolean_T c4_isNewLineChar_data[29]);
static __global__ void c4_eML_blk_kernel_kernel34(const boolean_T
  c4_isNewLineChar_data[29], int32_T *c4_nz);
static __global__ void c4_eML_blk_kernel_kernel35(const int32_T
  c4_isNewLineChar_size[2], const boolean_T c4_isNewLineChar_data[29], int32_T
  c4_vlen, int32_T *c4_nz);
static __global__ void c4_eML_blk_kernel_kernel36(const uint8_T
  c4_thisTextU16_data[29], const int32_T c4_i5, const int32_T c4_i7, uint16_T
  c4_thisCharcodes_1b_data[29]);
static __global__ void c4_eML_blk_kernel_kernel37(const int8_T c4_iv1[261],
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], const
  int32_T c4_thisCharcodes_1b_size[2], int8_T c4_x_data[29]);
static __global__ void c4_eML_blk_kernel_kernel38(const int8_T c4_iv1[261],
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], real_T
  *c4_lenEndSegment);
static __global__ void c4_eML_blk_kernel_kernel39(const int32_T c4_x_size[2],
  const int8_T c4_x_data[29], int32_T c4_vlen, real_T *c4_lenEndSegment);
static __global__ void c4_eML_blk_kernel_kernel40(const uint16_T c4_uv[256],
  const uint16_T c4_thisCharcodes_1b_data[29], const int32_T
  c4_thisCharcodes_1b_size[2], boolean_T c4_isNewLineChar_data[29]);
static __global__ void c4_eML_blk_kernel_kernel41(const boolean_T
  c4_isNewLineChar_data[29], int32_T *c4_nz);
static __global__ void c4_eML_blk_kernel_kernel42(const int32_T
  c4_isNewLineChar_size[2], const boolean_T c4_isNewLineChar_data[29], int32_T
  c4_vlen, int32_T *c4_nz);
static __global__ void c4_eML_blk_kernel_kernel43(const uint8_T
  c4_thisTextU16_data[29], const int32_T c4_thisTextU16_size[2], uint16_T
  c4_thisCharcodes_1b_data[29]);
static __global__ void c4_eML_blk_kernel_kernel44(const int8_T c4_iv1[261],
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], const
  int32_T c4_thisCharcodes_1b_size[2], int8_T c4_x_data[29]);
static __global__ void c4_eML_blk_kernel_kernel45(const int8_T c4_iv1[261],
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], real_T
  *c4_y);
static __global__ void c4_eML_blk_kernel_kernel46(const int32_T c4_x_size[2],
  const int8_T c4_x_data[29], int32_T c4_vlen, real_T *c4_y);
static __global__ void c4_eML_blk_kernel_kernel47(const uint16_T c4_uv[256],
  const uint16_T c4_thisCharcodes_1b_data[29], const int32_T
  c4_thisCharcodes_1b_size[2], boolean_T c4_isNewLineChar_data[29]);
static __global__ void c4_eML_blk_kernel_kernel48(const boolean_T
  c4_isNewLineChar_data[29], int32_T *c4_nz);
static __global__ void c4_eML_blk_kernel_kernel49(const int32_T
  c4_isNewLineChar_size[2], const boolean_T c4_isNewLineChar_data[29], int32_T
  c4_vlen, int32_T *c4_nz);
static __global__ void c4_eML_blk_kernel_kernel50(const uint8_T
  c4_thisTextU16_data[29], const int32_T c4_thisTextU16_size[2], boolean_T
  c4_isNewLineChar_data[29]);
static __global__ void c4_eML_blk_kernel_kernel51(const uint16_T c4_uv[256],
  const int32_T c4_i, const uint8_T c4_thisTextU16_data[29], uint16_T
  *c4_thisGlyphIdx_1b);
static __global__ void c4_eML_blk_kernel_kernel52(const uint8_T c4_uv4[10664],
  const int32_T c4_i10, const int32_T c4_i11, uint8_T c4_uv4_data[10664]);
static __global__ void c4_eML_blk_kernel_kernel53(const uint8_T c4_uv4_data
  [10664], const int32_T c4_uv4_size[2], const int8_T c4_outsize[2], uint8_T
  c4_b_uv4_data[144]);
static __global__ void c4_eML_blk_kernel_kernel54(const int8_T c4_outsize_idx_1,
  int8_T c4_outsize[2]);
static __global__ void c4_eML_blk_kernel_kernel55(const uint8_T c4_uv4_data[144],
  const int8_T c4_outsize[2], const int32_T c4_thisGlyphCut_float_size[2], const
  int32_T c4_i12, const int32_T c4_i13, const int32_T c4_i14, const int32_T
  c4_i15, real_T c4_thisGlyphCut_float_data[132]);
static __global__ void c4_eML_blk_kernel_kernel56(const int8_T c4_iv1[261],
  const uint16_T *c4_thisGlyphIdx_1b, int32_T *c4_q1);
static __global__ void c4_eML_blk_kernel_kernel57(const real32_T c4_prevpt[2],
  const int32_T c4_k, real32_T c4_pts[112]);
static __global__ void c4_eML_blk_kernel_kernel58(const real32_T c4_b_ltPts[56],
  const int32_T c4_k, const int32_T c4_b_k, real32_T c4_pts[112]);
static __global__ void c4_eML_blk_kernel_kernel59(const real32_T c4_pts[112],
  int32_T c4_positionOut[112]);
static __global__ void c4_eML_blk_kernel_kernel60(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel61(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel62(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel63(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel64(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel65(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel66(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel67(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel68(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel69(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel70(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel71(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel72(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel73(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel74(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel75(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel76(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel77(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel78(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel79(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel80(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel81(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel82(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel83(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel84(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel85(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel86(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel87(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel88(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel89(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel90(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel91(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel92(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel93(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel94(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel95(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel96(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel97(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel98(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel99(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel100(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel101(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel102(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel103(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel104(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel105(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel106(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel107(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel108(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel109(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel110(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel111(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel112(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel113(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel114(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel115(const
  c4_emxArray_cell_wrap_21_28 *c4_pos, int32_T c4_positionOut_data[112]);
static __global__ void c4_eML_blk_kernel_kernel116(real32_T c4_c_In[921600],
  real32_T c4_b_I[921600]);
static __global__ void c4_eML_blk_kernel_kernel117(uint8_T c4_pixCount[640]);
static __global__ void c4_eML_blk_kernel_kernel118(const int32_T
  c4_positionOut_data[112], int32_T c4_b_positionOut_data[112]);
static __global__ void c4_eML_blk_kernel_kernel119(const int8_T c4_color[84],
  real32_T c4_b_color[84]);
static __global__ void c4_eML_blk_kernel_kernel120(const real32_T c4_prevpt[2],
  const int32_T c4_k, real32_T c4_pts[112]);
static __global__ void c4_eML_blk_kernel_kernel121(const real32_T c4_b_rtPts[56],
  const int32_T c4_k, const int32_T c4_b_k, real32_T c4_pts[112]);
static __global__ void c4_eML_blk_kernel_kernel122(const real32_T c4_pts[112],
  int32_T c4_positionOut[112]);
static __global__ void c4_eML_blk_kernel_kernel123(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel124(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel125(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel126(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel127(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel128(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel129(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel130(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel131(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel132(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel133(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel134(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel135(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel136(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel137(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel138(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel139(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel140(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel141(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel142(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel143(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel144(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel145(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel146(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel147(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel148(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel149(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel150(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel151(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel152(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel153(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel154(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel155(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel156(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel157(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel158(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel159(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel160(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel161(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel162(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel163(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel164(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel165(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel166(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel167(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel168(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel169(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel170(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel171(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel172(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel173(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel174(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel175(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel176(c4_emxArray_cell_wrap_21_28
  *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel177(const int32_T c4_positionOut
  [112], c4_emxArray_cell_wrap_21_28 *c4_pos);
static __global__ void c4_eML_blk_kernel_kernel178(const
  c4_emxArray_cell_wrap_21_28 *c4_pos, int32_T c4_positionOut_data[112]);
static __global__ void c4_eML_blk_kernel_kernel179(real32_T c4_c_In[921600],
  real32_T c4_b_I[921600]);
static __global__ void c4_eML_blk_kernel_kernel180(uint8_T c4_pixCount[640]);
static __global__ void c4_eML_blk_kernel_kernel181(const int32_T
  c4_positionOut_data[112], int32_T c4_b_positionOut_data[112]);
static __global__ void c4_eML_blk_kernel_kernel182(const int8_T c4_color[84],
  real32_T c4_b_color[84]);
static __global__ void c4_eML_blk_kernel_kernel183(const real32_T c4_c_In[921600],
  real32_T c4_b_I[921600]);
static __global__ void c4_eML_blk_kernel_kernel184(const real32_T c4_b_ltPts[56],
  int32_T c4_position[56]);
static __global__ void c4_eML_blk_kernel_kernel185(const int8_T c4_fv1[84],
  real32_T c4_color[84]);
static __global__ void c4_eML_blk_kernel_kernel186(uint8_T c4_pixCount[640]);
static __global__ void c4_eML_blk_kernel_kernel187(const real32_T c4_c_In[921600],
  real32_T c4_b_I[921600]);
static __global__ void c4_eML_blk_kernel_kernel188(const real32_T c4_b_rtPts[56],
  int32_T c4_position[56]);
static __global__ void c4_eML_blk_kernel_kernel189(const int8_T c4_fv1[84],
  real32_T c4_color[84]);
static __global__ void c4_eML_blk_kernel_kernel190(uint8_T c4_pixCount[640]);
static void init_dsm_address_info(SFc4_LaneDetectionInstanceStruct
  *chartInstance);
static void init_simulink_io_address(SFc4_LaneDetectionInstanceStruct
  *chartInstance);

/* Function Definitions */
static void initialize_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
  emlrtLicenseCheckR2012b(chartInstance->c4_fEmlrtCtx,
    "distrib_computing_toolbox", 2);
  emlrtLicenseCheckR2012b(chartInstance->c4_fEmlrtCtx,
    "video_and_image_blockset", 2);
  sim_mode_is_external(chartInstance->S);
  _sfTime_ = sf_get_time(chartInstance->S);
  chartInstance->c4_is_active_c4_LaneDetection = 0U;
  cudaGetLastError();
  cudaMalloc(&chartInstance->c4_b_gpu_iv1, 8UL);
  cudaMalloc(&chartInstance->c4_f_gpu_vlen, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_rtPts, 224UL);
  cudaMalloc(&chartInstance->c4_gpu_thisTextU16_data, 29UL);
  cudaMalloc(&chartInstance->c4_b_gpu_uv4_data, 144UL);
  cudaMalloc(&chartInstance->c4_gpu_i7, 4UL);
  cudaMalloc(&chartInstance->c4_b_gpu_color, 336UL);
  cudaMalloc(&chartInstance->c4_h_gpu_vlen, 4UL);
  cudaMalloc(&chartInstance->c4_b_gpu_positionOut_data, 448UL);
  cudaMalloc(&chartInstance->c4_c_gpu_vlen, 4UL);
  cudaMalloc(&chartInstance->c4_b_gpu_vlen, 4UL);
  cudaMalloc(&chartInstance->c4_b_gpu_textLocAndWidth_size, 4UL);
  cudaMalloc(&chartInstance->c4_b_gpu_color_data, 240UL);
  cudaMalloc(&chartInstance->c4_b_gpu_textLocAndWidth_data, 320UL);
  cudaMalloc(&chartInstance->c4_d_gpu_nz, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_i3, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_position_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_textLocAndWidth_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_i12, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_i13, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_lenFirstSegment, 8UL);
  cudaMalloc(&chartInstance->c4_b_gpu_nz, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_thisGlyphCut_float_data, 1056UL);
  cudaMalloc(&chartInstance->c4_gpu_cv8, 6UL);
  cudaMalloc(&chartInstance->c4_gpu_uv4_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_vlen, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_isNewLineChar_data, 29UL);
  cudaMalloc(&chartInstance->c4_gpu_thisGlyphIdx_1b, 2UL);
  cudaMalloc(&chartInstance->c4_b_gpu_color_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_textPosition_data, 160UL);
  cudaMalloc(&chartInstance->c4_gpu_y, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_color, 84UL);
  cudaMalloc(&chartInstance->c4_c_gpu_k, 4UL);
  cudaMalloc(&chartInstance->c4_d_gpu_k, 4UL);
  cudaMalloc(&chartInstance->c4_e_gpu_vlen, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_i6, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_x_size, 8UL);
  cudaMalloc(&chartInstance->c4_c_gpu_nz, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_color_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_positionOut, 448UL);
  cudaMalloc(&chartInstance->c4_gpu_uv4_data, 10664UL);
  cudaMalloc(&chartInstance->c4_gpu_position_data, 320UL);
  cudaMalloc(&chartInstance->c4_gpu_i25, 4UL);
  cudaMalloc(&chartInstance->c4_c_gpu_color_data, 60UL);
  cudaMalloc(&chartInstance->c4_gpu_uv4, 10664UL);
  cudaMalloc(&chartInstance->c4_gpu_i, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_i11, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_lenEndSegment, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_q1, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_shapeHeight_data, 80UL);
  cudaMalloc(&chartInstance->c4_gpu_k, 4UL);
  cudaMalloc(&chartInstance->c4_g_gpu_vlen, 4UL);
  cudaMalloc(&chartInstance->c4_d_gpu_vlen, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_pixCount, 640UL);
  cudaMalloc(&chartInstance->c4_b_gpu_k, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_x_data, 29UL);
  cudaMalloc(&chartInstance->c4_gpu_i34, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_i15, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_i14, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_i10, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_iv, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_outsize, 2UL);
  cudaMalloc(&chartInstance->c4_gpu_idxNewlineChar_data, 29UL);
  cudaMalloc(&chartInstance->c4_gpu_fv1, 84UL);
  cudaMalloc(&chartInstance->c4_gpu_bboxes_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_lenThisSegment, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_pos, 564UL);
  cudaMalloc(&chartInstance->c4_gpu_positionOut_data, 448UL);
  cudaMalloc(&chartInstance->c4_gpu_nz, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_textLocAndWidth_data, 80UL);
  cudaMalloc(&chartInstance->c4_gpu_cv7, 6UL);
  cudaMalloc(&chartInstance->c4_gpu_ltPts, 224UL);
  cudaMalloc(&chartInstance->c4_gpu_pts, 448UL);
  cudaMalloc(&chartInstance->c4_gpu_i8, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_outsize_idx_1, 1UL);
  cudaMalloc(&chartInstance->c4_gpu_uv, 512UL);
  cudaMalloc(&chartInstance->c4_gpu_thisGlyphCut_float_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_i1, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_prevpt, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_I, 3686400UL);
  cudaMalloc(&chartInstance->c4_gpu_shapeWidth_data, 80UL);
  cudaMalloc(&chartInstance->c4_gpu_ii_data, 29UL);
  cudaMalloc(&chartInstance->c4_gpu_textPosition_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_iv1, 261UL);
  cudaMalloc(&chartInstance->c4_gpu_thisCharcodes_1b_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_position, 224UL);
  cudaMalloc(&chartInstance->c4_gpu_i28, 4UL);
  cudaMalloc(&chartInstance->c4_gpu_thisTextU16_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_bboxes_data, 80U * sizeof(real_T));
  cudaMalloc(&chartInstance->c4_b_gpu_In, 3686400UL);
  cudaMalloc(&chartInstance->c4_gpu_str1, 30UL);
  cudaMalloc(&chartInstance->c4_gpu_thisCharcodes_1b_data, 58UL);
  cudaMalloc(&chartInstance->c4_gpu_i5, 4UL);
  cudaMalloc(&chartInstance->c4_b_gpu_outsize, 2UL);
  cudaMalloc(&chartInstance->c4_gpu_textColor_data, 60UL);
  cudaMalloc(&chartInstance->c4_gpu_ii_size, 8UL);
  cudaMalloc(&chartInstance->c4_gpu_color_data, 60UL);
  cudaMalloc(&chartInstance->c4_gpu_isNewLineChar_size, 8UL);
}

static void initialize_params_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct *
  chartInstance)
{
}

static void enable_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void disable_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
  _sfTime_ = sf_get_time(chartInstance->S);
}

static void c4_do_animation_call_c4_LaneDetection
  (SFc4_LaneDetectionInstanceStruct *chartInstance)
{
  sfDoAnimationWrapper(chartInstance->S, false, true);
  sfDoAnimationWrapper(chartInstance->S, false, false);
}

static void ext_mode_exec_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static const mxArray *get_sim_state_c4_LaneDetection
  (SFc4_LaneDetectionInstanceStruct *chartInstance)
{
  const mxArray *c4_b_y = NULL;
  const mxArray *c4_c_y = NULL;
  const mxArray *c4_st;
  const mxArray *c4_y = NULL;
  c4_st = NULL;
  c4_y = NULL;
  sf_mex_assign(&c4_y, sf_mex_createcellmatrix(2, 1), false);
  c4_b_y = NULL;
  sf_mex_assign(&c4_b_y, sf_mex_create("y", *chartInstance->c4_b_In, 1, 0U, 1U,
    0U, 3, 480, 640, 3), false);
  sf_mex_setcell(c4_y, 0, c4_b_y);
  c4_c_y = NULL;
  sf_mex_assign(&c4_c_y, sf_mex_create("y",
    &chartInstance->c4_is_active_c4_LaneDetection, 3, 0U, 0U, 0U, 0), false);
  sf_mex_setcell(c4_y, 1, c4_c_y);
  sf_mex_assign(&c4_st, c4_y, false);
  return c4_st;
}

static void set_sim_state_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c4_st)
{
  const mxArray *c4_u;
  c4_u = sf_mex_dup(c4_st);
  c4_emlrt_marshallIn(chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 0)), "In", *
                      chartInstance->c4_b_In);
  chartInstance->c4_is_active_c4_LaneDetection = c4_c_emlrt_marshallIn
    (chartInstance, sf_mex_dup(sf_mex_getcell(c4_u, 1)),
     "is_active_c4_LaneDetection");
  sf_mex_destroy(&c4_u);
  sf_mex_destroy(&c4_st);
}

static void sf_gateway_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
  int32_T c4_iv[2];
  int32_T c4_iv1[2];
  int32_T c4_i;
  int32_T c4_i1;
  int32_T c4_i2;
  int32_T c4_i3;
  real32_T c4_b_fv[56];
  real32_T c4_fv1[56];
  chartInstance->c4_JITTransitionAnimation[0] = 0U;
  _sfTime_ = sf_get_time(chartInstance->S);
  cudaMemcpyToSymbol(c4_gpu_In, *chartInstance->c4_b_In, 3686400UL, 0UL,
                     cudaMemcpyHostToDevice);
  cudaMemcpyToSymbol(c4_gpu_In_1, *chartInstance->c4_In, 3686400UL, 0UL,
                     cudaMemcpyHostToDevice);
  c4_sf_gateway_c4_LaneDetection_kernel1<<<dim3(1800U, 1U, 1U), dim3(512U, 1U,
    1U)>>>();
  cudaMemcpyFromSymbol(*chartInstance->c4_b_In, c4_gpu_In, 3686400UL, 0UL,
                       cudaMemcpyDeviceToHost);
  for (c4_i = 0; c4_i < 56; c4_i++) {
    c4_b_fv[c4_i] = (*chartInstance->c4_ltPts)[c4_i];
  }

  for (c4_i1 = 0; c4_i1 < 56; c4_i1++) {
    c4_fv1[c4_i1] = (*chartInstance->c4_rtPts)[c4_i1];
  }

  for (c4_i2 = 0; c4_i2 < 2; c4_i2++) {
    c4_iv[c4_i2] = (*chartInstance->c4_bboxes_sizes)[c4_i2];
  }

  for (c4_i3 = 0; c4_i3 < 2; c4_i3++) {
    c4_iv1[c4_i3] = (*chartInstance->c4_scores_sizes)[c4_i3];
  }

  c4_eML_blk_kernel(chartInstance, *chartInstance->c4_laneFound, c4_b_fv, c4_fv1,
                    *chartInstance->c4_bboxes_data, c4_iv,
                    *chartInstance->c4_scores_data, c4_iv1,
                    *chartInstance->c4_b_In);
  c4_do_animation_call_c4_LaneDetection(chartInstance);
}

static void mdl_start_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static void mdl_terminate_c4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
  cudaError_t c4_errCode;
  cudaFree(*chartInstance->c4_b_gpu_outsize);
  cudaFree(chartInstance->c4_g_gpu_vlen);
  cudaFree(chartInstance->c4_gpu_lenThisSegment);
  cudaFree(*chartInstance->c4_gpu_textPosition_data);
  cudaFree(*chartInstance->c4_gpu_pts);
  cudaFree(chartInstance->c4_gpu_k);
  cudaFree(*chartInstance->c4_gpu_uv);
  cudaFree(chartInstance->c4_gpu_i14);
  cudaFree(*chartInstance->c4_b_gpu_In);
  cudaFree(chartInstance->c4_gpu_i28);
  cudaFree(chartInstance->c4_gpu_lenFirstSegment);
  cudaFree(chartInstance->c4_d_gpu_nz);
  cudaFree(chartInstance->c4_c_gpu_vlen);
  cudaFree(*chartInstance->c4_gpu_positionOut);
  cudaFree(chartInstance->c4_gpu_i25);
  cudaFree(chartInstance->c4_h_gpu_vlen);
  cudaFree(*chartInstance->c4_b_gpu_color_data);
  cudaFree(*chartInstance->c4_gpu_iv1);
  cudaFree(*chartInstance->c4_gpu_x_size);
  cudaFree(*chartInstance->c4_gpu_iv);
  cudaFree(chartInstance->c4_gpu_i34);
  cudaFree(*chartInstance->c4_gpu_pixCount);
  cudaFree(chartInstance->c4_b_gpu_k);
  cudaFree(chartInstance->c4_gpu_i);
  cudaFree(*chartInstance->c4_gpu_I);
  cudaFree(*chartInstance->c4_gpu_ltPts);
  cudaFree(*chartInstance->c4_gpu_prevpt);
  cudaFree(*chartInstance->c4_gpu_uv4);
  cudaFree(chartInstance->c4_c_gpu_nz);
  cudaFree(*chartInstance->c4_gpu_position);
  cudaFree(*chartInstance->c4_b_gpu_color);
  cudaFree(*chartInstance->c4_gpu_textPosition_size);
  cudaFree(chartInstance->c4_gpu_i11);
  cudaFree(*chartInstance->c4_gpu_position_size);
  cudaFree(*chartInstance->c4_b_gpu_positionOut_data);
  cudaFree(*chartInstance->c4_gpu_color_size);
  cudaFree(*chartInstance->c4_b_gpu_color_size);
  cudaFree(chartInstance->c4_gpu_y);
  cudaFree(chartInstance->c4_gpu_lenEndSegment);
  cudaFree(*chartInstance->c4_gpu_str1);
  cudaFree(*chartInstance->c4_b_gpu_textLocAndWidth_data);
  cudaFree(chartInstance->c4_d_gpu_vlen);
  cudaFree(*chartInstance->c4_gpu_cv7);
  cudaFree(chartInstance->c4_b_gpu_vlen);
  cudaFree(chartInstance->c4_gpu_pos);
  cudaFree(chartInstance->c4_gpu_i10);
  cudaFree(*chartInstance->c4_gpu_isNewLineChar_data);
  cudaFree(*chartInstance->c4_gpu_shapeHeight_data);
  cudaFree(*chartInstance->c4_gpu_outsize);
  cudaFree(chartInstance->c4_b_gpu_nz);
  cudaFree(*chartInstance->c4_gpu_position_data);
  cudaFree(chartInstance->c4_gpu_i3);
  cudaFree(*chartInstance->c4_gpu_color_data);
  cudaFree(*chartInstance->c4_gpu_shapeWidth_data);
  cudaFree(*chartInstance->c4_gpu_fv1);
  cudaFree(*chartInstance->c4_gpu_cv8);
  cudaFree(*chartInstance->c4_gpu_textLocAndWidth_data);
  cudaFree(*chartInstance->c4_gpu_color);
  cudaFree(chartInstance->c4_gpu_i13);
  cudaFree(chartInstance->c4_gpu_vlen);
  cudaFree(chartInstance->c4_gpu_bboxes_data);
  cudaFree(*chartInstance->c4_gpu_textColor_data);
  cudaFree(chartInstance->c4_gpu_i8);
  cudaFree(*chartInstance->c4_gpu_rtPts);
  cudaFree(chartInstance->c4_gpu_thisGlyphIdx_1b);
  cudaFree(*chartInstance->c4_gpu_thisCharcodes_1b_data);
  cudaFree(*chartInstance->c4_gpu_x_data);
  cudaFree(*chartInstance->c4_gpu_thisGlyphCut_float_size);
  cudaFree(chartInstance->c4_gpu_q1);
  cudaFree(*chartInstance->c4_gpu_isNewLineChar_size);
  cudaFree(chartInstance->c4_gpu_i1);
  cudaFree(*chartInstance->c4_gpu_positionOut_data);
  cudaFree(*chartInstance->c4_gpu_idxNewlineChar_data);
  cudaFree(*chartInstance->c4_b_gpu_iv1);
  cudaFree(*chartInstance->c4_gpu_thisCharcodes_1b_size);
  cudaFree(*chartInstance->c4_gpu_thisGlyphCut_float_data);
  cudaFree(*chartInstance->c4_gpu_ii_data);
  cudaFree(chartInstance->c4_c_gpu_k);
  cudaFree(chartInstance->c4_gpu_i12);
  cudaFree(chartInstance->c4_gpu_outsize_idx_1);
  cudaFree(*chartInstance->c4_gpu_thisTextU16_data);
  cudaFree(chartInstance->c4_gpu_nz);
  cudaFree(chartInstance->c4_gpu_i6);
  cudaFree(chartInstance->c4_gpu_i5);
  cudaFree(*chartInstance->c4_gpu_uv4_data);
  cudaFree(*chartInstance->c4_gpu_thisTextU16_size);
  cudaFree(*chartInstance->c4_b_gpu_textLocAndWidth_size);
  cudaFree(chartInstance->c4_gpu_i7);
  cudaFree(*chartInstance->c4_gpu_ii_size);
  cudaFree(chartInstance->c4_gpu_i15);
  cudaFree(chartInstance->c4_d_gpu_k);
  cudaFree(*chartInstance->c4_gpu_uv4_size);
  cudaFree(*chartInstance->c4_gpu_bboxes_size);
  cudaFree(*chartInstance->c4_b_gpu_uv4_data);
  cudaFree(*chartInstance->c4_gpu_textLocAndWidth_size);
  cudaFree(chartInstance->c4_e_gpu_vlen);
  cudaFree(chartInstance->c4_f_gpu_vlen);
  cudaFree(*chartInstance->c4_c_gpu_color_data);
  c4_errCode = cudaGetLastError();
  if (c4_errCode != cudaSuccess) {
    emlrtThinCUDAError(c4_errCode, cudaGetErrorName(c4_errCode),
                       cudaGetErrorString(c4_errCode), "SimGPUErrorChecks",
                       chartInstance->c4_fEmlrtCtx);
  }
}

static void mdl_setup_runtime_resources_c4_LaneDetection
  (SFc4_LaneDetectionInstanceStruct *chartInstance)
{
  setLegacyDebuggerFlag(chartInstance->S, false);
  setDebuggerFlag(chartInstance->S, false);
  sim_mode_is_external(chartInstance->S);
}

static void mdl_cleanup_runtime_resources_c4_LaneDetection
  (SFc4_LaneDetectionInstanceStruct *chartInstance)
{
}

static void initSimStructsc4_LaneDetection(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
}

const mxArray *sf_c4_LaneDetection_get_eml_resolved_functions_info()
{
  const mxArray *c4_nameCaptureInfo = NULL;
  const char_T *c4_data[4] = {
    "789c6360f4f465646060e003e299f50c0c126c0c60c00ba11804a03413032a409767c441c3002b030b8a3e90fc14a07dfd507e727e5e496a4509849397989b0a"
    "d799929f9b9997985712525990ca50945a9c9f53969a029649cbcc490dc9cc4d0d46e6f88178b96e4852700e480ac476ce484dce0e2ecd6528ca2846b8300799",
    "030e0f1048c0e15f1602e1810ed0c3035d1dccbe0a32ed83992f4ec03e987c4e625e6a7c596a4666724e6a7c625e5e7e496249667e1ec5fe86d9cb86d31d1099"
    "e2ccbcf49c5484bf6750689f214efb50e5a35d63f533f27353f533729373f41d5d40e9ae283f471f4778e8e542f5110a177e22dd892b7ff0327080e9cdfb59e2",
    "e9691f0c8c14fb28cd5f6238ec13409337f68b32ab0acd4834aef0764d2dae08724fab2c77f140b82380803d84dcc180834f6bf301e8bc58b6",
    "" };

  c4_nameCaptureInfo = NULL;
  emlrtNameCaptureMxArrayR2016a(&c4_data[0], 1584U, &c4_nameCaptureInfo);
  return c4_nameCaptureInfo;
}

static void c4_emlrt_marshallIn(SFc4_LaneDetectionInstanceStruct *chartInstance,
  const mxArray *c4_c_In, const char_T *c4_identifier, real32_T c4_y[921600])
{
  emlrtMsgIdentifier c4_thisId;
  c4_thisId.fIdentifier = const_cast<const char_T *>(c4_identifier);
  c4_thisId.fParent = NULL;
  c4_thisId.bParentIsCell = false;
  c4_b_emlrt_marshallIn(chartInstance, sf_mex_dup(c4_c_In), &c4_thisId, c4_y);
  sf_mex_destroy(&c4_c_In);
}

static void c4_b_emlrt_marshallIn(SFc4_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId,
  real32_T c4_y[921600])
{
  int32_T c4_i;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), chartInstance->c4_fv, 0, 1, 0U, 1,
                0U, 3, 480, 640, 3);
  for (c4_i = 0; c4_i < 921600; c4_i++) {
    c4_y[c4_i] = chartInstance->c4_fv[c4_i];
  }

  sf_mex_destroy(&c4_u);
}

static uint8_T c4_c_emlrt_marshallIn(SFc4_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c4_b_is_active_c4_LaneDetection, const char_T
  *c4_identifier)
{
  emlrtMsgIdentifier c4_thisId;
  uint8_T c4_y;
  c4_thisId.fIdentifier = const_cast<const char_T *>(c4_identifier);
  c4_thisId.fParent = NULL;
  c4_thisId.bParentIsCell = false;
  c4_y = c4_d_emlrt_marshallIn(chartInstance, sf_mex_dup
    (c4_b_is_active_c4_LaneDetection), &c4_thisId);
  sf_mex_destroy(&c4_b_is_active_c4_LaneDetection);
  return c4_y;
}

static uint8_T c4_d_emlrt_marshallIn(SFc4_LaneDetectionInstanceStruct
  *chartInstance, const mxArray *c4_u, const emlrtMsgIdentifier *c4_parentId)
{
  uint8_T c4_b_u;
  uint8_T c4_y;
  sf_mex_import(c4_parentId, sf_mex_dup(c4_u), &c4_b_u, 1, 3, 0U, 0, 0U, 0);
  c4_y = c4_b_u;
  sf_mex_destroy(&c4_u);
  return c4_y;
}

static void c4_eML_blk_kernel(SFc4_LaneDetectionInstanceStruct *chartInstance,
  boolean_T c4_b_laneFound, real32_T c4_b_ltPts[56], real32_T c4_b_rtPts[56],
  real_T c4_b_bboxes_data[], int32_T c4_bboxes_size[2], real32_T
  c4_b_scores_data[], int32_T c4_scores_size[2], real32_T c4_c_In[921600])
{
  static int16_T c4_uv3[261] = { 0, 0, 0, 56, 56, 74, 86, 158, 224, 296, 368,
    377, 421, 465, 489, 545, 551, 558, 560, 615, 678, 732, 786, 840, 903, 957,
    1020, 1074, 1128, 1191, 1205, 1223, 1279, 1303, 1359, 1404, 1494, 1575, 1629,
    1701, 1773, 1827, 1872, 1944, 2007, 2025, 2080, 2143, 2197, 2278, 2341, 2422,
    2476, 2586, 2649, 2703, 2775, 2838, 2910, 3009, 3081, 3153, 3216, 3249, 3304,
    3337, 3379, 3385, 3393, 3442, 3496, 3538, 3601, 3643, 3688, 3751, 3805, 3823,
    3867, 3921, 3939, 4009, 4051, 4100, 4154, 4217, 4245, 4287, 4327, 4369, 4418,
    4488, 4537, 4600, 4649, 4693, 4715, 4759, 6416, 6515, 6722, 6890, 7390, 7906,
    8394, 8745, 8675, 8815, 8955, 8885, 9018, 9165, 9285, 9225, 9345, 9405, 9499,
    9459, 9539, 9589, 9695, 9825, 9755, 9895, 10035, 9965, 10263, 10203, 10323,
    10383, 0, 5322, 4791, 4845, 5049, 0, 5515, 8621, 5286, 5119, 0, 5453, 5115,
    0, 6623, 8061, 0, 5337, 0, 0, 4955, 5461, 0, 0, 0, 0, 0, 5209, 5619, 0, 9095,
    10154, 5939, 4773, 5255, 0, 0, 0, 0, 5225, 5639, 0, 4773, 5984, 6308, 7798,
    0, 0, 0, 0, 0, 0, 0, 0, 10098, 0, 10587, 0, 0, 4899, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 6200, 6962, 6092, 7034, 6818, 7148, 7196, 7256, 7100, 7582, 7690, 0, 7474,
    8226, 8310, 8142, 0, 0, 0, 0, 0, 0, 0, 5583, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5027,
    7300, 9625, 8471, 10437, 8567, 10521, 0, 8005, 5595, 5393, 5423, 5759, 5669,
    5849, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5279, 5316, 5581 };

  static uint16_T c4_uv[256] = { 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 3U, 4U, 5U, 6U, 7U, 8U, 9U, 10U, 11U, 12U, 13U, 14U, 15U, 16U, 17U, 18U,
    19U, 20U, 21U, 22U, 23U, 24U, 25U, 26U, 27U, 28U, 29U, 30U, 31U, 32U, 33U,
    34U, 35U, 36U, 37U, 38U, 39U, 40U, 41U, 42U, 43U, 44U, 45U, 46U, 47U, 48U,
    49U, 50U, 51U, 52U, 53U, 54U, 55U, 56U, 57U, 58U, 59U, 60U, 61U, 62U, 63U,
    64U, 65U, 66U, 67U, 68U, 69U, 70U, 71U, 72U, 73U, 74U, 75U, 76U, 77U, 78U,
    79U, 80U, 81U, 82U, 83U, 84U, 85U, 86U, 87U, 88U, 89U, 90U, 91U, 92U, 93U,
    94U, 95U, 96U, 97U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    172U, 163U, 132U, 133U, 189U, 150U, 232U, 134U, 142U, 139U, 157U, 169U, 164U,
    258U, 138U, 259U, 131U, 147U, 242U, 243U, 141U, 151U, 136U, 260U, 222U, 241U,
    158U, 170U, 245U, 244U, 246U, 162U, 173U, 201U, 199U, 174U, 98U, 99U, 144U,
    100U, 203U, 101U, 200U, 202U, 207U, 204U, 205U, 206U, 233U, 102U, 211U, 208U,
    209U, 175U, 103U, 240U, 145U, 214U, 212U, 213U, 104U, 235U, 237U, 137U, 106U,
    105U, 107U, 109U, 108U, 110U, 160U, 111U, 113U, 112U, 114U, 115U, 117U, 116U,
    118U, 119U, 234U, 120U, 122U, 121U, 123U, 125U, 124U, 184U, 161U, 127U, 126U,
    128U, 129U, 236U, 238U, 186U };

  static char_T c4_cv8[6] = "%0.5g";
  static int8_T c4_b_iv1[261] = { 9, 0, 0, 4, 4, 4, 8, 8, 8, 8, 3, 4, 4, 6, 10,
    4, 7, 4, 6, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 4, 4, 10, 10, 10, 5, 10, 8, 7, 8,
    9, 7, 6, 9, 9, 3, 4, 8, 6, 10, 9, 9, 7, 9, 8, 6, 8, 8, 8, 10, 8, 7, 7, 4, 6,
    4, 8, 6, 7, 7, 8, 6, 8, 7, 4, 7, 7, 3, 4, 7, 3, 11, 7, 7, 8, 8, 5, 6, 4, 7,
    6, 9, 7, 6, 7, 4, 4, 4, 8, 8, 8, 8, 7, 9, 9, 8, 7, 7, 7, 7, 7, 7, 6, 7, 7, 7,
    7, 3, 3, 3, 3, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 0, 5, 8, 8, 8, 0, 8, 7, 8, 10,
    0, 7, 7, 0, 11, 9, 0, 10, 0, 0, 8, 8, 0, 0, 0, 0, 0, 6, 6, 0, 10, 7, 5, 4,
    10, 0, 0, 0, 0, 6, 6, 0, 4, 8, 8, 9, 0, 0, 0, 0, 0, 0, 0, 0, 10, 0, 6, 0, 0,
    8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 7, 8, 7, 7, 3, 3, 3, 3, 9, 9, 0, 9, 8, 8, 8,
    0, 0, 0, 0, 0, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 9, 7, 7, 6, 7, 8, 0,
    10, 5, 5, 5, 10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 6, 4 };

  static int8_T c4_iv2[261] = { 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 1,
    0, 1, 1, 0, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 1, 1, 1,
    0, 1, 1, -2, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0,
    2, 0, 1, 0, 0, 0, 0, 0, 1, 1, -1, 1, 1, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0,
    0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    -1, -1, 0, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1, 1, 1, 0, 1, 1, 1, 0, 0, 2,
    2, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 1, 0, 1, 1, 0, -1, 0, -1, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0,
    0, 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 1, 1, 0, 1, 1, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1 };

  static int8_T c4_iv3[261] = { 8, 0, 0, 0, 9, 9, 9, 10, 9, 9, 9, 9, 9, 9, 7, 1,
    4, 1, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 7, 7, 7, 5, 7, 9, 9, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 0,
    10, 7, 9, 7, 9, 7, 9, 7, 9, 9, 9, 9, 9, 7, 7, 7, 7, 7, 7, 7, 8, 7, 7, 7, 7,
    7, 7, 9, 9, 9, 5, 11, 12, 9, 12, 12, 11, 11, 10, 10, 10, 9, 10, 11, 7, 10,
    10, 10, 9, 10, 10, 10, 9, 10, 10, 10, 10, 9, 10, 10, 10, 10, 9, 0, 9, 9, 9,
    9, 0, 9, 9, 9, 9, 0, 10, 9, 0, 9, 9, 0, 7, 0, 0, 9, 7, 0, 0, 0, 0, 0, 9, 9,
    0, 7, 7, 7, 7, 5, 0, 0, 0, 0, 6, 6, 0, 0, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 0,
    7, 0, 9, 0, 0, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 11, 12, 12, 12, 11,
    12, 12, 12, 0, 12, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 9, 9, 10, 12, 10, 9, 9, 0, 7, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 4, 10, 4 };

  static int8_T c4_uv1[261] = { 8, 0, 0, 0, 9, 3, 9, 11, 9, 9, 3, 11, 11, 4, 7,
    3, 1, 1, 11, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 7, 9, 7, 3, 7, 9, 9, 9, 9, 9, 9,
    9, 9, 9, 9, 9, 11, 9, 9, 9, 9, 9, 9, 11, 9, 9, 9, 9, 9, 9, 9, 9, 9, 11, 11,
    11, 6, 1, 2, 7, 9, 7, 9, 7, 9, 9, 9, 9, 11, 9, 9, 7, 7, 7, 9, 9, 7, 7, 8, 7,
    7, 7, 7, 9, 7, 11, 11, 11, 2, 11, 12, 12, 12, 12, 11, 11, 10, 10, 10, 9, 10,
    11, 10, 10, 10, 10, 9, 10, 10, 10, 9, 10, 10, 10, 10, 9, 10, 10, 10, 10, 9,
    0, 3, 9, 9, 11, 0, 11, 9, 5, 9, 0, 2, 1, 0, 9, 9, 0, 7, 0, 0, 9, 9, 0, 0, 0,
    0, 0, 4, 4, 0, 7, 7, 9, 9, 3, 0, 0, 0, 0, 5, 5, 0, 0, 12, 12, 12, 0, 0, 0, 0,
    0, 0, 0, 0, 7, 0, 11, 0, 0, 7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 12, 12, 12, 11, 12,
    12, 12, 11, 12, 12, 12, 0, 12, 12, 12, 12, 0, 0, 0, 0, 0, 0, 0, 3, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 11, 9, 10, 12, 12, 9, 11, 0, 7, 6, 6, 6, 9, 9, 9, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1 };

  static int8_T c4_uv2[261] = { 7, 0, 0, 0, 2, 4, 8, 6, 8, 8, 3, 4, 4, 6, 8, 2,
    7, 2, 5, 7, 6, 6, 6, 7, 6, 7, 6, 6, 7, 2, 2, 8, 8, 8, 5, 10, 9, 6, 8, 8, 6,
    5, 8, 7, 2, 5, 7, 6, 9, 7, 9, 6, 10, 7, 6, 8, 7, 8, 11, 8, 8, 7, 3, 5, 3, 7,
    6, 4, 7, 6, 6, 7, 6, 5, 7, 6, 2, 4, 6, 2, 10, 6, 7, 6, 7, 4, 6, 5, 6, 7, 10,
    7, 7, 7, 4, 2, 4, 7, 9, 9, 8, 6, 7, 9, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 4,
    4, 5, 4, 6, 7, 7, 7, 7, 7, 6, 6, 6, 6, 0, 5, 6, 6, 6, 0, 6, 6, 6, 10, 0, 4,
    4, 0, 11, 9, 0, 8, 0, 0, 8, 6, 0, 0, 0, 0, 0, 4, 5, 0, 10, 7, 5, 2, 8, 0, 0,
    0, 0, 6, 6, 0, 0, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 8, 0, 7, 0, 0, 8, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 9, 6, 9, 6, 6, 4, 5, 4, 4, 9, 9, 0, 9, 7, 7, 7, 0, 0, 0, 0,
    0, 0, 0, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 10, 7, 8, 7, 6, 6, 0, 8, 4, 5, 5,
    10, 10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 7, 6, 2 };

  static int8_T c4_color[84] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static int8_T c4_fv1[84] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

  static int8_T c4_b_fv[3] = { 1, 1, 0 };

  static uint8_T c4_uv4[10664] = { 60U, 96U, 96U, 96U, 96U, 96U, 60U, 96U, 0U,
    0U, 0U, 0U, 0U, 96U, 96U, 0U, 0U, 0U, 0U, 0U, 96U, 96U, 0U, 0U, 0U, 0U, 0U,
    96U, 96U, 0U, 0U, 0U, 0U, 0U, 96U, 96U, 0U, 0U, 0U, 0U, 0U, 96U, 96U, 0U, 0U,
    0U, 0U, 0U, 96U, 108U, 96U, 96U, 96U, 96U, 96U, 108U, 176U, 120U, 176U, 119U,
    172U, 115U, 165U, 108U, 158U, 101U, 151U, 94U, 144U, 87U, 0U, 0U, 176U, 120U,
    83U, 201U, 79U, 205U, 71U, 189U, 67U, 193U, 58U, 177U, 54U, 181U, 0U, 0U, 0U,
    185U, 6U, 117U, 75U, 0U, 0U, 0U, 3U, 187U, 0U, 172U, 20U, 0U, 0U, 0U, 48U,
    143U, 0U, 192U, 0U, 0U, 74U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 109U, 0U, 0U, 172U, 19U, 110U, 79U,
    0U, 0U, 214U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 225U, 0U, 0U, 69U, 123U, 3U, 187U, 0U, 0U, 0U, 0U, 125U, 67U,
    49U, 143U, 0U, 0U, 0U, 0U, 178U, 13U, 104U, 87U, 0U, 0U, 0U, 0U, 0U, 108U,
    40U, 0U, 0U, 3U, 155U, 245U, 230U, 104U, 0U, 89U, 202U, 130U, 73U, 164U, 7U,
    109U, 164U, 108U, 40U, 0U, 0U, 26U, 231U, 195U, 40U, 0U, 0U, 0U, 38U, 207U,
    190U, 29U, 0U, 0U, 0U, 108U, 163U, 223U, 7U, 0U, 0U, 108U, 40U, 230U, 38U,
    145U, 76U, 117U, 107U, 230U, 7U, 42U, 181U, 246U, 221U, 67U, 0U, 0U, 0U,
    108U, 40U, 0U, 0U, 80U, 234U, 221U, 50U, 0U, 0U, 106U, 126U, 211U, 40U, 81U,
    171U, 0U, 50U, 179U, 2U, 212U, 39U, 90U, 170U, 15U, 194U, 21U, 0U, 83U, 235U,
    222U, 52U, 170U, 60U, 0U, 0U, 0U, 0U, 0U, 114U, 117U, 0U, 0U, 0U, 0U, 0U,
    57U, 173U, 48U, 219U, 236U, 86U, 0U, 19U, 194U, 17U, 166U, 89U, 38U, 218U,
    1U, 176U, 53U, 0U, 167U, 92U, 45U, 220U, 123U, 109U, 0U, 0U, 49U, 221U, 236U,
    88U, 0U, 0U, 83U, 227U, 235U, 93U, 0U, 0U, 0U, 1U, 240U, 84U, 72U, 238U, 0U,
    0U, 0U, 0U, 230U, 69U, 73U, 208U, 0U, 0U, 0U, 3U, 171U, 218U, 208U, 42U, 0U,
    0U, 12U, 199U, 146U, 230U, 87U, 0U, 91U, 198U, 107U, 179U, 0U, 86U, 229U,
    17U, 111U, 162U, 129U, 183U, 0U, 0U, 172U, 181U, 177U, 80U, 55U, 252U, 110U,
    13U, 41U, 240U, 215U, 1U, 0U, 81U, 209U, 247U, 228U, 160U, 247U, 101U, 77U,
    MAX_uint8_T, 10U, 53U, 241U, 0U, 28U, 216U, 0U, 0U, 0U, 48U, 94U, 0U, 19U,
    216U, 35U, 0U, 150U, 115U, 0U, 6U, 239U, 26U, 0U, 43U, 240U, 0U, 0U, 65U,
    228U, 0U, 0U, 43U, 240U, 0U, 0U, 6U, 239U, 25U, 0U, 0U, 150U, 113U, 0U, 0U,
    19U, 216U, 35U, 0U, 0U, 48U, 94U, 107U, 34U, 0U, 0U, 51U, 209U, 8U, 0U, 0U,
    142U, 122U, 0U, 0U, 53U, 219U, 0U, 0U, 9U, MAX_uint8_T, 17U, 0U, 0U, 252U,
    40U, 0U, 9U, MAX_uint8_T, 17U, 0U, 52U, 219U, 0U, 0U, 140U, 122U, 0U, 50U,
    209U, 8U, 0U, 107U, 34U, 0U, 0U, 0U, 0U, 124U, 69U, 0U, 0U, 58U, 208U, 141U,
    142U, 217U, 9U, 0U, 19U, 131U, 142U, 2U, 0U, 0U, 123U, 57U, 109U, 70U, 0U,
    0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U,
    0U, 168U, 52U, 0U, 0U, 0U, 180U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U, 0U, 168U, 52U, 0U, 0U,
    0U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U,
    212U, 160U, 105U, 147U, 186U, 51U, 32U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 212U, 160U, 0U, 0U, 0U, 220U,
    10U, 0U, 0U, 45U, 186U, 0U, 0U, 0U, 118U, 113U, 0U, 0U, 0U, 191U, 41U, 0U,
    0U, 13U, 218U, 0U, 0U, 0U, 80U, 151U, 0U, 0U, 0U, 153U, 78U, 0U, 0U, 0U,
    219U, 12U, 0U, 0U, 42U, 189U, 0U, 0U, 0U, 115U, 116U, 0U, 0U, 0U, 188U, 44U,
    0U, 0U, 0U, 0U, 6U, 155U, 244U, 222U, 79U, 0U, 0U, 130U, 192U, 23U, 68U,
    239U, 29U, 4U, 235U, 65U, 0U, 0U, 165U, 134U, 31U, MAX_uint8_T, 18U, 0U, 0U,
    119U, 182U, 53U, MAX_uint8_T, 4U, 0U, 0U, 105U, 204U, 31U, MAX_uint8_T, 18U,
    0U, 0U, 120U, 181U, 3U, 234U, 64U, 0U, 0U, 168U, 133U, 0U, 128U, 190U, 23U,
    70U, 240U, 29U, 0U, 6U, 156U, 245U, 223U, 79U, 0U, 1U, 60U, 149U, 157U, 0U,
    0U, 40U, 178U, 186U, 180U, 0U, 0U, 0U, 0U, 116U, 180U, 0U, 0U, 0U, 0U, 116U,
    180U, 0U, 0U, 0U, 0U, 116U, 180U, 0U, 0U, 0U, 0U, 116U, 180U, 0U, 0U, 0U, 0U,
    116U, 180U, 0U, 0U, 0U, 0U, 116U, 180U, 0U, 0U, 48U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 112U, 200U, MAX_uint8_T, 251U, 208U,
    65U, 0U, 0U, 0U, 8U, 120U, 240U, 10U, 0U, 0U, 0U, 18U, MAX_uint8_T, 42U, 0U,
    0U, 0U, 81U, 237U, 9U, 0U, 0U, 35U, 224U, 76U, 0U, 0U, 43U, 220U, 73U, 0U,
    0U, 30U, 226U, 64U, 0U, 0U, 0U, 187U, 148U, 0U, 0U, 0U, 0U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 40U, 168U, MAX_uint8_T,
    249U, 209U, 67U, 0U, 0U, 0U, 8U, 133U, 221U, 0U, 0U, 0U, 0U, 59U, 234U, 0U,
    0U, 4U, 38U, 183U, 111U, 0U, 0U, 252U, MAX_uint8_T, 183U, 26U, 0U, 0U, 3U,
    28U, 139U, 231U, 14U, 0U, 0U, 0U, 5U, 253U, 60U, 0U, 0U, 11U, 122U, 244U,
    20U, 200U, MAX_uint8_T, 246U, 202U, 67U, 0U, 0U, 0U, 0U, 9U, 211U, 156U, 0U,
    0U, 0U, 0U, 158U, 188U, 156U, 0U, 0U, 0U, 96U, 153U, 104U, 156U, 0U, 0U, 44U,
    200U, 9U, 104U, 156U, 0U, 12U, 205U, 41U, 0U, 104U, 156U, 0U, 97U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 196U, 0U,
    0U, 0U, 0U, 124U, 156U, 0U, 0U, 0U, 0U, 0U, 124U, 156U, 0U, 0U, 0U, 0U, 0U,
    124U, 156U, 0U, 100U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 0U,
    100U, 160U, 0U, 0U, 0U, 0U, 100U, 160U, 0U, 0U, 0U, 0U, 100U, 253U, 224U,
    147U, 16U, 0U, 0U, 8U, 49U, 194U, 180U, 0U, 0U, 0U, 0U, 43U, MAX_uint8_T,
    16U, 0U, 0U, 0U, 35U, MAX_uint8_T, 25U, 0U, 0U, 18U, 167U, 204U, 0U, 140U,
    MAX_uint8_T, 236U, 173U, 30U, 0U, 0U, 0U, 117U, 225U, 254U, MAX_uint8_T, 72U,
    0U, 100U, 215U, 48U, 2U, 0U, 0U, 0U, 219U, 80U, 0U, 0U, 0U, 0U, 17U,
    MAX_uint8_T, 117U, 226U, 237U, 144U, 3U, 43U, MAX_uint8_T, 169U, 20U, 45U,
    229U, 107U, 29U, MAX_uint8_T, 40U, 0U, 0U, 133U, 173U, 3U, 241U, 63U, 0U, 0U,
    129U, 158U, 0U, 140U, 195U, 27U, 39U, 224U, 69U, 0U, 10U, 162U, 244U, 224U,
    102U, 0U, 180U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 224U, 0U,
    0U, 0U, 0U, 136U, 145U, 0U, 0U, 0U, 36U, 225U, 15U, 0U, 0U, 0U, 180U, 94U,
    0U, 0U, 0U, 74U, 213U, 2U, 0U, 0U, 4U, 216U, 85U, 0U, 0U, 0U, 101U, 228U, 3U,
    0U, 0U, 0U, 208U, 134U, 0U, 0U, 0U, 21U, MAX_uint8_T, 67U, 0U, 0U, 0U, 1U,
    137U, 233U, 241U, 163U, 8U, 89U, 197U, 22U, 27U, 206U, 105U, 114U, 176U, 0U,
    0U, 171U, 86U, 18U, 216U, 174U, 121U, 159U, 2U, 12U, 179U, 191U, 252U, 125U,
    1U, 161U, 128U, 0U, 58U, 227U, 127U, 236U, 62U, 0U, 0U, 103U, 203U, 194U,
    171U, 20U, 26U, 187U, 149U, 32U, 180U, 242U, 234U, 154U, 12U, 0U, 13U, 160U,
    239U, 226U, 94U, 0U, 0U, 159U, 154U, 14U, 71U, 245U, 45U, 2U, 246U, 31U, 0U,
    0U, 160U, 148U, 7U, 252U, 37U, 0U, 0U, 137U, 187U, 0U, 185U, 162U, 15U, 53U,
    229U, 196U, 0U, 26U, 184U, 245U, 188U, 164U, 165U, 0U, 0U, 0U, 0U, 0U, 196U,
    101U, 0U, 0U, 0U, 15U, 121U, 221U, 6U, 0U, 164U, MAX_uint8_T, 246U, 181U,
    34U, 0U, 176U, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 176U, 120U,
    176U, 120U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 176U, 119U, 96U, 104U,
    145U, 26U, 0U, 0U, 0U, 0U, 0U, 36U, 160U, 56U, 0U, 0U, 0U, 34U, 158U, 210U,
    87U, 1U, 0U, 32U, 156U, 212U, 91U, 2U, 0U, 0U, 63U, 245U, 180U, 6U, 0U, 0U,
    0U, 0U, 0U, 32U, 157U, 212U, 91U, 2U, 0U, 0U, 0U, 0U, 0U, 34U, 159U, 211U,
    90U, 2U, 0U, 0U, 0U, 0U, 0U, 36U, 161U, 56U, 180U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 180U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 60U, 144U, 102U, 5U, 0U, 0U, 0U, 0U, 0U, 25U, 146U,
    215U, 99U, 4U, 0U, 0U, 0U, 0U, 0U, 27U, 149U, 215U, 97U, 4U, 0U, 0U, 0U, 0U,
    0U, 59U, 243U, 185U, 8U, 0U, 0U, 28U, 149U, 215U, 98U, 4U, 0U, 27U, 148U,
    216U, 100U, 4U, 0U, 0U, 0U, 145U, 103U, 5U, 0U, 0U, 0U, 0U, 0U, 200U,
    MAX_uint8_T, 245U, 188U, 34U, 0U, 0U, 16U, 174U, 179U, 0U, 0U, 0U, 128U,
    193U, 0U, 0U, 21U, 232U, 83U, 0U, 0U, 179U, 107U, 0U, 0U, 67U, 201U, 0U, 0U,
    0U, 125U, 162U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 136U, 160U, 0U, 0U, 0U, 0U,
    0U, 77U, 186U, 240U, 244U, 185U, 50U, 0U, 0U, 2U, 155U, 178U, 61U, 10U, 22U,
    89U, 217U, 48U, 0U, 125U, 124U, 3U, 143U, 240U, MAX_uint8_T, 111U, 43U, 173U,
    19U, 171U, 0U, 131U, 117U, 13U, 165U, 55U, 0U, 175U, 87U, 81U, 10U, 197U, 0U,
    43U, 238U, 6U, 9U, 167U, 110U, 65U, 54U, 189U, 44U, 185U, 201U, 20U, 164U,
    75U, 73U, 143U, 22U, 230U, 208U, 51U, 233U, 220U, 97U, 0U, 2U, 197U, 138U,
    35U, 3U, 0U, 0U, 0U, 0U, 0U, 0U, 12U, 138U, 221U, 253U, MAX_uint8_T, 77U, 0U,
    0U, 0U, 0U, 0U, 0U, 166U, 236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U, 83U,
    0U, 0U, 0U, 0U, 0U, 106U, 187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U, 101U,
    85U, 251U, 23U, 0U, 0U, 0U, 46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U, 0U,
    145U, 186U, 0U, 0U, 167U, 211U, 0U, 0U, 6U, 236U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 51U, 0U, 85U, 235U, 10U, 0U, 0U, 4U,
    224U, 147U, 0U, 183U, 124U, 0U, 0U, 0U, 0U, 108U, 236U, 6U, 224U,
    MAX_uint8_T, 250U, 211U, 74U, 0U, 224U, 88U, 21U, 154U, 239U, 1U, 224U, 88U,
    0U, 66U, 249U, 4U, 224U, 89U, 32U, 184U, 137U, 0U, 224U, MAX_uint8_T,
    MAX_uint8_T, 192U, 16U, 0U, 224U, 89U, 32U, 157U, 220U, 14U, 224U, 88U, 0U,
    2U, 244U, 88U, 224U, 88U, 9U, 81U, MAX_uint8_T, 65U, 224U, MAX_uint8_T, 254U,
    229U, 128U, 0U, 0U, 0U, 98U, 207U, 248U, MAX_uint8_T, MAX_uint8_T, 160U, 0U,
    125U, 240U, 93U, 18U, 0U, 0U, 0U, 19U, 248U, 99U, 0U, 0U, 0U, 0U, 0U, 77U,
    MAX_uint8_T, 13U, 0U, 0U, 0U, 0U, 0U, 96U, 244U, 0U, 0U, 0U, 0U, 0U, 0U, 78U,
    MAX_uint8_T, 15U, 0U, 0U, 0U, 0U, 0U, 21U, 250U, 108U, 0U, 0U, 0U, 0U, 0U,
    0U, 135U, 244U, 104U, 23U, 0U, 0U, 0U, 0U, 1U, 107U, 211U, 249U, MAX_uint8_T,
    MAX_uint8_T, 164U, 224U, MAX_uint8_T, MAX_uint8_T, 247U, 215U, 119U, 2U, 0U,
    224U, 88U, 2U, 19U, 87U, 239U, 135U, 0U, 224U, 88U, 0U, 0U, 0U, 98U, 247U,
    21U, 224U, 88U, 0U, 0U, 0U, 11U, MAX_uint8_T, 68U, 224U, 88U, 0U, 0U, 0U, 0U,
    244U, 89U, 224U, 88U, 0U, 0U, 0U, 15U, MAX_uint8_T, 65U, 224U, 88U, 0U, 0U,
    0U, 102U, 240U, 10U, 224U, 88U, 0U, 23U, 94U, 240U, 106U, 0U, 224U,
    MAX_uint8_T, MAX_uint8_T, 246U, 201U, 86U, 0U, 0U, 224U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 248U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 96U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 56U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 248U, 224U,
    88U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 224U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 100U, 224U, 88U, 0U, 0U, 0U, 224U,
    88U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 0U, 100U,
    208U, 248U, MAX_uint8_T, MAX_uint8_T, 160U, 0U, 128U, 240U, 93U, 18U, 0U, 0U,
    0U, 19U, 248U, 99U, 0U, 0U, 0U, 0U, 0U, 77U, MAX_uint8_T, 13U, 0U, 0U, 0U,
    0U, 0U, 96U, 244U, 0U, 0U, 0U, 0U, 0U, 0U, 78U, MAX_uint8_T, 15U, 0U, 0U, 0U,
    152U, 164U, 21U, 250U, 106U, 0U, 0U, 0U, 152U, 164U, 0U, 138U, 243U, 104U,
    24U, 17U, 171U, 164U, 0U, 1U, 108U, 210U, 248U, 242U, 206U, 101U, 224U, 88U,
    0U, 0U, 0U, 136U, 180U, 224U, 88U, 0U, 0U, 0U, 136U, 180U, 224U, 88U, 0U, 0U,
    0U, 136U, 180U, 224U, 88U, 0U, 0U, 0U, 136U, 180U, 224U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 180U, 224U, 88U, 0U, 0U,
    0U, 136U, 180U, 224U, 88U, 0U, 0U, 0U, 136U, 180U, 224U, 88U, 0U, 0U, 0U,
    136U, 180U, 224U, 88U, 0U, 0U, 0U, 136U, 180U, 228U, 88U, 228U, 88U, 228U,
    88U, 228U, 88U, 228U, 88U, 228U, 88U, 228U, 88U, 228U, 88U, 228U, 88U, 0U,
    0U, 0U, 148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U,
    0U, 148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U, 0U,
    148U, 168U, 0U, 0U, 0U, 148U, 168U, 0U, 0U, 0U, 156U, 151U, 0U, 0U, 20U,
    218U, 94U, 20U, MAX_uint8_T, 236U, 147U, 3U, 224U, 72U, 0U, 0U, 157U, 158U,
    0U, 224U, 72U, 0U, 104U, 204U, 8U, 0U, 224U, 72U, 56U, 229U, 30U, 0U, 0U,
    224U, 95U, 225U, 67U, 0U, 0U, 0U, 224U, 204U, 215U, 7U, 0U, 0U, 0U, 224U,
    82U, 205U, 165U, 0U, 0U, 0U, 224U, 72U, 27U, 230U, 127U, 0U, 0U, 224U, 72U,
    0U, 54U, 246U, 90U, 0U, 224U, 72U, 0U, 0U, 91U, 247U, 58U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U,
    0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 64U, 224U, 245U, 12U, 0U, 0U, 0U, 124U,
    MAX_uint8_T, 56U, 224U, 247U, 94U, 0U, 0U, 0U, 216U, 249U, 56U, 224U, 171U,
    187U, 0U, 0U, 58U, 189U, 236U, 56U, 224U, 78U, 252U, 27U, 0U, 153U, 93U,
    236U, 56U, 224U, 44U, 197U, 117U, 8U, 227U, 11U, 236U, 56U, 224U, 44U, 104U,
    209U, 88U, 158U, 0U, 236U, 56U, 224U, 44U, 18U, 249U, 217U, 63U, 0U, 236U,
    56U, 224U, 44U, 0U, 174U, 222U, 1U, 0U, 236U, 56U, 224U, 44U, 0U, 0U, 0U, 0U,
    0U, 236U, 56U, 224U, 163U, 0U, 0U, 0U, 80U, 192U, 224U, MAX_uint8_T, 68U, 0U,
    0U, 80U, 192U, 224U, 183U, 220U, 8U, 0U, 80U, 192U, 224U, 54U, 223U, 133U,
    0U, 80U, 192U, 224U, 44U, 73U, 250U, 43U, 80U, 192U, 224U, 44U, 0U, 168U,
    196U, 81U, 192U, 224U, 44U, 0U, 23U, 239U, 183U, 192U, 224U, 44U, 0U, 0U,
    102U, MAX_uint8_T, 192U, 224U, 44U, 0U, 0U, 1U, 195U, 192U, 0U, 0U, 101U,
    213U, 249U, 231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U,
    5U, 17U, 247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U,
    0U, 0U, 0U, 183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U,
    21U, 245U, 91U, 0U, 122U, 236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U,
    215U, 250U, 230U, 148U, 14U, 0U, 224U, MAX_uint8_T, 252U, 229U, 141U, 1U,
    224U, 88U, 8U, 73U, 253U, 75U, 224U, 88U, 0U, 0U, 239U, 89U, 224U, 88U, 20U,
    129U, 239U, 26U, 224U, MAX_uint8_T, 236U, 184U, 52U, 0U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 0U, 0U, 101U, 213U, 249U, 231U, 148U, 14U, 0U, 0U, 0U, 122U,
    235U, 80U, 14U, 45U, 195U, 197U, 4U, 0U, 17U, 247U, 91U, 0U, 0U, 0U, 21U,
    244U, 90U, 0U, 76U, MAX_uint8_T, 11U, 0U, 0U, 0U, 0U, 183U, 158U, 0U, 96U,
    243U, 0U, 0U, 0U, 0U, 0U, 160U, 178U, 0U, 76U, MAX_uint8_T, 12U, 0U, 0U, 0U,
    0U, 184U, 157U, 0U, 16U, 245U, 94U, 0U, 0U, 0U, 21U, 244U, 89U, 0U, 0U, 118U,
    236U, 78U, 13U, 46U, 196U, 197U, 5U, 0U, 0U, 0U, 103U, 217U, 250U, 250U,
    215U, 15U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 25U, 187U, 212U, 98U, 15U, 0U, 0U, 0U,
    0U, 0U, 0U, 1U, 87U, 208U, 57U, 224U, MAX_uint8_T, 254U, 231U, 123U, 0U, 0U,
    224U, 88U, 9U, 91U, MAX_uint8_T, 49U, 0U, 224U, 88U, 0U, 4U, 251U, 70U, 0U,
    224U, 88U, 22U, 138U, 225U, 10U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, 214U,
    29U, 0U, 0U, 224U, 88U, 55U, 248U, 47U, 0U, 0U, 224U, 88U, 0U, 152U, 205U,
    4U, 0U, 224U, 88U, 0U, 16U, 232U, 122U, 0U, 224U, 88U, 0U, 0U, 91U, 248U,
    42U, 0U, 101U, 225U, MAX_uint8_T, MAX_uint8_T, 84U, 45U, 242U, 53U, 1U, 0U,
    0U, 78U, 233U, 9U, 0U, 0U, 0U, 11U, 216U, 206U, 55U, 0U, 0U, 0U, 18U, 161U,
    253U, 148U, 5U, 0U, 0U, 0U, 62U, 233U, 143U, 0U, 0U, 0U, 0U, 116U, 208U, 0U,
    0U, 0U, 22U, 187U, 151U, 104U, MAX_uint8_T, MAX_uint8_T, 232U, 153U, 12U,
    228U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 124U, 0U, 0U, 0U, 208U, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U,
    104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U,
    0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U,
    0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 104U, 0U, 0U,
    0U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U,
    72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U,
    0U, 0U, 208U, 68U, 240U, 75U, 0U, 0U, 0U, 210U, 64U, 214U, 105U, 0U, 0U, 1U,
    235U, 39U, 137U, 216U, 43U, 11U, 110U, 221U, 4U, 12U, 150U, 229U, 243U, 197U,
    51U, 0U, 154U, 157U, 0U, 0U, 0U, 0U, 117U, 158U, 61U, 241U, 9U, 0U, 0U, 0U,
    206U, 69U, 1U, 223U, 87U, 0U, 0U, 40U, 231U, 3U, 0U, 131U, 180U, 0U, 0U,
    129U, 145U, 0U, 0U, 38U, 250U, 21U, 0U, 218U, 55U, 0U, 0U, 0U, 201U, 109U,
    52U, 221U, 0U, 0U, 0U, 0U, 108U, 202U, 142U, 132U, 0U, 0U, 0U, 0U, 20U, 250U,
    240U, 42U, 0U, 0U, 0U, 0U, 0U, 178U, 208U, 0U, 0U, 0U, 221U, 83U, 0U, 0U,
    130U, 231U, 0U, 0U, 0U, 226U, 29U, 157U, 146U, 0U, 0U, 188U, MAX_uint8_T,
    31U, 0U, 39U, 215U, 0U, 93U, 209U, 0U, 3U, 227U, 213U, 87U, 0U, 108U, 146U,
    0U, 30U, 254U, 18U, 49U, 183U, 156U, 143U, 0U, 177U, 76U, 0U, 0U, 222U, 79U,
    108U, 125U, 100U, 199U, 4U, 236U, 12U, 0U, 0U, 159U, 142U, 166U, 67U, 44U,
    248U, 66U, 194U, 0U, 0U, 0U, 95U, 205U, 221U, 12U, 2U, 241U, 183U, 124U, 0U,
    0U, 0U, 31U, 254U, 206U, 0U, 0U, 188U, MAX_uint8_T, 55U, 0U, 0U, 0U, 0U,
    224U, 149U, 0U, 0U, 132U, 239U, 3U, 0U, 0U, 118U, 235U, 19U, 0U, 0U, 91U,
    203U, 3U, 5U, 212U, 157U, 0U, 20U, 230U, 48U, 0U, 0U, 62U, 253U, 60U, 165U,
    136U, 0U, 0U, 0U, 0U, 162U, 231U, 220U, 9U, 0U, 0U, 0U, 0U, 48U, MAX_uint8_T,
    135U, 0U, 0U, 0U, 0U, 0U, 174U, 204U, 243U, 28U, 0U, 0U, 0U, 80U, 214U, 9U,
    201U, 172U, 0U, 0U, 14U, 227U, 59U, 0U, 49U, 252U, 73U, 0U, 151U, 149U, 0U,
    0U, 0U, 143U, 220U, 8U, 159U, 199U, 1U, 0U, 0U, 39U, 231U, 22U, 24U, 242U,
    92U, 0U, 0U, 190U, 102U, 0U, 0U, 120U, 228U, 11U, 96U, 198U, 1U, 0U, 0U, 7U,
    221U, 150U, 231U, 46U, 0U, 0U, 0U, 0U, 81U, MAX_uint8_T, 139U, 0U, 0U, 0U,
    0U, 0U, 0U, MAX_uint8_T, 56U, 0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 56U, 0U,
    0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 56U, 0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T,
    56U, 0U, 0U, 0U, 40U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 176U, 0U, 0U, 0U, 0U, 38U, 246U, 90U, 0U, 0U, 0U, 2U, 197U,
    176U, 0U, 0U, 0U, 0U, 114U, 237U, 24U, 0U, 0U, 0U, 37U, 246U, 91U, 0U, 0U,
    0U, 2U, 196U, 177U, 0U, 0U, 0U, 0U, 113U, 238U, 25U, 0U, 0U, 0U, 36U, 246U,
    93U, 0U, 0U, 0U, 0U, 112U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 176U, 216U, MAX_uint8_T, 120U, 216U, 44U, 0U, 216U,
    44U, 0U, 216U, 44U, 0U, 216U, 44U, 0U, 216U, 44U, 0U, 216U, 44U, 0U, 216U,
    44U, 0U, 216U, 44U, 0U, 216U, 44U, 0U, 216U, MAX_uint8_T, 120U, 184U, 47U,
    0U, 0U, 0U, 111U, 120U, 0U, 0U, 0U, 38U, 193U, 0U, 0U, 0U, 0U, 217U, 14U, 0U,
    0U, 0U, 149U, 82U, 0U, 0U, 0U, 76U, 155U, 0U, 0U, 0U, 10U, 220U, 0U, 0U, 0U,
    0U, 186U, 44U, 0U, 0U, 0U, 114U, 117U, 0U, 0U, 0U, 41U, 190U, 0U, 0U, 0U, 0U,
    218U, 13U, 143U, MAX_uint8_T, 191U, 0U, 67U, 191U, 0U, 67U, 191U, 0U, 67U,
    191U, 0U, 67U, 191U, 0U, 67U, 191U, 0U, 67U, 191U, 0U, 67U, 191U, 0U, 67U,
    191U, 0U, 67U, 191U, 143U, MAX_uint8_T, 191U, 0U, 0U, 0U, 110U, 25U, 0U, 0U,
    0U, 0U, 17U, 235U, 151U, 0U, 0U, 0U, 0U, 137U, 115U, 208U, 36U, 0U, 0U, 26U,
    215U, 7U, 81U, 167U, 0U, 0U, 153U, 96U, 0U, 0U, 198U, 48U, 36U, 208U, 2U, 0U,
    0U, 64U, 183U, 128U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    124U, 64U, 225U, 23U, 0U, 0U, 77U, 175U, 0U, 0U, 60U, 200U, 244U, 183U, 9U,
    0U, 0U, 149U, 44U, 23U, 229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U,
    91U, 205U, 242U, MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U,
    84U, 226U, 27U, 47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U,
    216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 113U, 200U, 243U,
    173U, 14U, 216U, 196U, 45U, 21U, 204U, 137U, 216U, 80U, 0U, 0U, 107U, 207U,
    216U, 80U, 0U, 0U, 90U, 222U, 216U, 125U, 0U, 0U, 119U, 191U, 216U, 246U,
    62U, 34U, 220U, 98U, 216U, 115U, 211U, 244U, 147U, 2U, 0U, 33U, 180U, 240U,
    MAX_uint8_T, 128U, 3U, 212U, 180U, 23U, 0U, 0U, 53U, MAX_uint8_T, 32U, 0U,
    0U, 0U, 78U, 251U, 0U, 0U, 0U, 0U, 46U, MAX_uint8_T, 33U, 0U, 0U, 0U, 0U,
    198U, 183U, 26U, 0U, 0U, 0U, 25U, 178U, 245U, MAX_uint8_T, 148U, 0U, 0U, 0U,
    0U, 0U, 196U, 104U, 0U, 0U, 0U, 0U, 0U, 196U, 104U, 0U, 47U, 208U, 241U,
    154U, 202U, 104U, 1U, 212U, 130U, 15U, 82U, 239U, 104U, 49U, 249U, 8U, 0U,
    0U, 196U, 104U, 79U, 229U, 0U, 0U, 0U, 196U, 104U, 64U, 243U, 2U, 0U, 0U,
    196U, 104U, 12U, 238U, 105U, 11U, 107U, 242U, 104U, 0U, 73U, 221U, 241U,
    130U, 196U, 104U, 0U, 35U, 194U, 241U, 167U, 9U, 2U, 211U, 108U, 15U, 183U,
    115U, 52U, 244U, 2U, 0U, 93U, 186U, 78U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 211U, 47U, 241U, 4U, 0U, 0U, 0U, 0U, 196U, 152U,
    21U, 0U, 0U, 0U, 22U, 170U, 239U, MAX_uint8_T, 208U, 0U, 57U, 218U, 253U,
    196U, 0U, 153U, 160U, 2U, 0U, 136U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    32U, 0U, 160U, 136U, 0U, 0U, 0U, 160U, 136U, 0U, 0U, 0U, 160U, 136U, 0U, 0U,
    0U, 160U, 136U, 0U, 0U, 0U, 160U, 136U, 0U, 0U, 0U, 160U, 136U, 0U, 0U, 0U,
    42U, 204U, 241U, 154U, 202U, 104U, 1U, 208U, 147U, 16U, 76U, 238U, 104U, 49U,
    251U, 14U, 0U, 0U, 196U, 104U, 79U, 230U, 0U, 0U, 0U, 196U, 104U, 61U, 243U,
    2U, 0U, 0U, 196U, 103U, 9U, 234U, 104U, 11U, 105U, 242U, 97U, 0U, 66U, 219U,
    242U, 136U, 203U, 79U, 0U, 0U, 0U, 7U, 77U, 242U, 22U, 0U, 194U, MAX_uint8_T,
    247U, 206U, 71U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U,
    216U, 103U, 184U, 245U, 171U, 2U, 216U, 222U, 66U, 21U, 239U, 67U, 216U, 95U,
    0U, 0U, 206U, 91U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U,
    92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U,
    0U, 0U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U,
    216U, 80U, 0U, 0U, 168U, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 128U, 0U, 0U,
    168U, 128U, 0U, 0U, 168U, 128U, 0U, 0U, 168U, 128U, 0U, 0U, 168U, 128U, 0U,
    0U, 168U, 128U, 0U, 0U, 170U, 124U, 0U, 10U, 211U, 88U, 232U, 248U, 176U, 7U,
    216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 98U,
    208U, 10U, 216U, 80U, 53U, 230U, 34U, 0U, 216U, 102U, 225U, 72U, 0U, 0U,
    216U, 203U, 224U, 12U, 0U, 0U, 216U, 85U, 192U, 176U, 1U, 0U, 216U, 80U, 19U,
    221U, 135U, 0U, 216U, 80U, 0U, 42U, 240U, 93U, 216U, 80U, 216U, 80U, 216U,
    80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U, 80U, 216U,
    117U, 198U, 245U, 128U, 14U, 184U, 243U, 133U, 0U, 216U, 245U, 80U, 36U,
    251U, 163U, 62U, 61U, 254U, 13U, 216U, 124U, 0U, 0U, 240U, 100U, 0U, 12U,
    MAX_uint8_T, 31U, 216U, 80U, 0U, 0U, 240U, 56U, 0U, 12U, MAX_uint8_T, 32U,
    216U, 80U, 0U, 0U, 240U, 56U, 0U, 12U, MAX_uint8_T, 32U, 216U, 80U, 0U, 0U,
    240U, 56U, 0U, 12U, MAX_uint8_T, 32U, 216U, 80U, 0U, 0U, 240U, 56U, 0U, 12U,
    MAX_uint8_T, 32U, 216U, 103U, 184U, 245U, 171U, 2U, 216U, 222U, 66U, 21U,
    239U, 67U, 216U, 95U, 0U, 0U, 206U, 91U, 216U, 80U, 0U, 0U, 204U, 92U, 216U,
    80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U,
    204U, 92U, 0U, 35U, 186U, 242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U,
    242U, 55U, 52U, 249U, 8U, 0U, 0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U,
    174U, 51U, 248U, 7U, 0U, 0U, 163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U,
    0U, 33U, 187U, 243U, 215U, 88U, 0U, 216U, 113U, 200U, 243U, 173U, 14U, 216U,
    196U, 45U, 21U, 204U, 137U, 216U, 80U, 0U, 0U, 107U, 207U, 216U, 80U, 0U, 0U,
    90U, 222U, 216U, 104U, 0U, 0U, 119U, 191U, 216U, 209U, 53U, 34U, 220U, 98U,
    216U, 99U, 198U, 244U, 147U, 2U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U,
    0U, 0U, 0U, 0U, 47U, 208U, 241U, 154U, 202U, 104U, 1U, 212U, 130U, 15U, 82U,
    239U, 104U, 49U, 249U, 8U, 0U, 0U, 196U, 104U, 79U, 229U, 0U, 0U, 0U, 196U,
    104U, 64U, 243U, 2U, 0U, 0U, 196U, 104U, 12U, 238U, 105U, 11U, 107U, 242U,
    104U, 0U, 73U, 221U, 241U, 130U, 196U, 104U, 0U, 0U, 0U, 0U, 0U, 196U, 104U,
    0U, 0U, 0U, 0U, 0U, 196U, 104U, 216U, 115U, 206U, 170U, 216U, 209U, 43U, 0U,
    216U, 87U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U,
    216U, 80U, 0U, 0U, 0U, 98U, 230U, MAX_uint8_T, 184U, 0U, 2U, 246U, 59U, 0U,
    0U, 0U, 0U, 226U, 157U, 20U, 0U, 0U, 0U, 46U, 198U, 244U, 107U, 0U, 0U, 0U,
    0U, 88U, 254U, 28U, 0U, 0U, 0U, 56U, 251U, 23U, 20U, MAX_uint8_T,
    MAX_uint8_T, 228U, 98U, 0U, 0U, 191U, 90U, 0U, 0U, 152U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 4U, 0U, 204U, 92U, 0U, 0U, 0U, 204U, 92U, 0U, 0U,
    0U, 204U, 92U, 0U, 0U, 0U, 203U, 93U, 0U, 0U, 0U, 183U, 154U, 4U, 0U, 0U,
    61U, 226U, 251U, 0U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U,
    72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 235U, 61U,
    0U, 4U, 230U, 72U, 212U, 112U, 21U, 162U, 246U, 72U, 80U, 234U, 227U, 88U,
    220U, 72U, 185U, 109U, 0U, 0U, 30U, 237U, 6U, 99U, 194U, 0U, 0U, 118U, 153U,
    0U, 18U, 248U, 25U, 0U, 208U, 61U, 0U, 0U, 182U, 107U, 40U, 224U, 1U, 0U, 0U,
    96U, 191U, 130U, 133U, 0U, 0U, 0U, 16U, 246U, 228U, 41U, 0U, 0U, 0U, 0U,
    179U, 205U, 0U, 0U, 0U, 203U, 83U, 0U, 12U, 250U, 94U, 0U, 5U, 236U, 12U,
    137U, 147U, 0U, 77U, 234U, 155U, 0U, 66U, 185U, 0U, 71U, 211U, 0U, 147U,
    115U, 216U, 0U, 140U, 109U, 0U, 10U, 248U, 20U, 210U, 11U, 233U, 22U, 213U,
    33U, 0U, 0U, 194U, 113U, 189U, 0U, 173U, 112U, 212U, 0U, 0U, 0U, 128U, 234U,
    120U, 0U, 110U, 234U, 137U, 0U, 0U, 0U, 61U, MAX_uint8_T, 50U, 0U, 47U,
    MAX_uint8_T, 61U, 0U, 0U, 30U, 240U, 76U, 0U, 3U, 205U, 79U, 0U, 95U, 232U,
    20U, 113U, 177U, 0U, 0U, 0U, 173U, 188U, 232U, 29U, 0U, 0U, 0U, 61U,
    MAX_uint8_T, 149U, 0U, 0U, 0U, 3U, 202U, 154U, 242U, 33U, 0U, 0U, 123U, 171U,
    0U, 155U, 195U, 2U, 44U, 228U, 22U, 0U, 12U, 220U, 119U, 187U, 131U, 0U, 0U,
    23U, 240U, 13U, 97U, 220U, 0U, 0U, 114U, 158U, 0U, 14U, 247U, 53U, 0U, 208U,
    59U, 0U, 0U, 172U, 142U, 45U, 216U, 0U, 0U, 0U, 82U, 229U, 142U, 117U, 0U,
    0U, 0U, 7U, 240U, 244U, 23U, 0U, 0U, 0U, 0U, 157U, 175U, 0U, 0U, 0U, 0U, 0U,
    179U, 76U, 0U, 0U, 0U, 0U, 54U, 229U, 3U, 0U, 0U, 0U, 24U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 0U, 0U, 0U, 2U,
    186U, 174U, 0U, 0U, 0U, 0U, 135U, 215U, 13U, 0U, 0U, 0U, 80U, 241U, 41U, 0U,
    0U, 0U, 39U, 241U, 83U, 0U, 0U, 0U, 12U, 214U, 136U, 0U, 0U, 0U, 0U, 72U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 40U, 0U,
    84U, 223U, 27U, 0U, 235U, 47U, 0U, 0U, 230U, 35U, 0U, 0U, 189U, 61U, 0U, 14U,
    205U, 23U, 0U, 220U, 155U, 0U, 0U, 14U, 204U, 23U, 0U, 0U, 190U, 61U, 0U, 0U,
    230U, 35U, 0U, 0U, 235U, 47U, 0U, 0U, 86U, 224U, 27U, 48U, 172U, 48U, 172U,
    48U, 172U, 48U, 172U, 48U, 172U, 48U, 172U, 48U, 172U, 48U, 172U, 48U, 172U,
    48U, 172U, 48U, 172U, 50U, 216U, 66U, 0U, 0U, 72U, 210U, 0U, 0U, 59U, 205U,
    0U, 0U, 85U, 164U, 0U, 0U, 43U, 190U, 9U, 0U, 0U, 179U, 195U, 0U, 42U, 189U,
    9U, 0U, 85U, 165U, 0U, 0U, 59U, 205U, 0U, 0U, 71U, 210U, 0U, 51U, 217U, 68U,
    0U, 6U, 189U, 242U, 170U, 66U, 27U, 222U, 79U, 168U, 21U, 111U, 206U, 244U,
    110U, 176U, 120U, 0U, 0U, 144U, 87U, 151U, 94U, 158U, 101U, 165U, 108U, 172U,
    115U, 176U, 119U, 176U, 120U, 0U, 0U, 28U, 120U, 0U, 0U, 0U, 102U, 223U,
    MAX_uint8_T, MAX_uint8_T, 32U, 75U, 236U, 80U, 120U, 0U, 0U, 170U, 150U, 28U,
    120U, 0U, 0U, 195U, 125U, 28U, 120U, 0U, 0U, 165U, 158U, 28U, 120U, 0U, 0U,
    65U, 246U, 102U, 120U, 0U, 0U, 0U, 97U, 224U, MAX_uint8_T, MAX_uint8_T, 32U,
    0U, 0U, 28U, 120U, 0U, 0U, 0U, 0U, 114U, 234U, MAX_uint8_T, 80U, 0U, 33U,
    243U, 34U, 0U, 0U, 0U, 76U, 213U, 0U, 0U, 0U, 0U, 84U, 212U, 0U, 0U, 0U, 96U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 24U, 0U, 0U, 84U, 211U, 0U, 0U, 0U,
    0U, 93U, 190U, 0U, 0U, 0U, 18U, 192U, 76U, 0U, 0U, 0U, 140U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 104U, 52U, 117U, 0U, 0U, 0U, 11U,
    160U, 0U, 0U, 165U, 174U, 242U, 220U, 191U, 67U, 0U, 0U, 139U, 147U, 14U,
    44U, 216U, 37U, 0U, 0U, 197U, 29U, 0U, 0U, 134U, 89U, 0U, 0U, 139U, 147U,
    14U, 42U, 215U, 37U, 0U, 0U, 165U, 174U, 242U, 221U, 191U, 67U, 0U, 52U,
    117U, 0U, 0U, 0U, 11U, 160U, 0U, 53U, 246U, 47U, 0U, 0U, 45U, 209U, 12U, 0U,
    132U, 208U, 5U, 10U, 207U, 49U, 0U, 0U, 5U, 207U, 131U, 155U, 111U, 0U, 0U,
    0U, 0U, 46U, 246U, 180U, 0U, 0U, 0U, 0U, 140U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 20U, 0U, 0U, 0U, 0U, 208U, 88U, 0U, 0U, 0U, 0U,
    140U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 0U, 0U, 0U,
    0U, 208U, 88U, 0U, 0U, 0U, 0U, 0U, 0U, 208U, 88U, 0U, 0U, 0U, 48U, 172U, 48U,
    172U, 48U, 172U, 48U, 172U, 0U, 0U, 0U, 0U, 0U, 0U, 48U, 172U, 48U, 172U,
    48U, 172U, 48U, 172U, 9U, 156U, 235U, MAX_uint8_T, MAX_uint8_T, 32U, 125U,
    179U, 23U, 0U, 0U, 0U, 135U, 171U, 6U, 0U, 0U, 0U, 18U, 237U, 227U, 109U, 6U,
    0U, 115U, 119U, 42U, 163U, 206U, 12U, 156U, 117U, 0U, 0U, 187U, 82U, 43U,
    226U, 128U, 17U, 198U, 36U, 0U, 18U, 135U, 232U, 184U, 0U, 0U, 0U, 0U, 20U,
    212U, 76U, 0U, 0U, 1U, 32U, 209U, 84U, 180U, MAX_uint8_T, 253U, 226U, 130U,
    1U, 252U, 8U, 168U, 88U, 0U, 0U, 45U, 174U, 239U, 246U, 199U, 79U, 0U, 0U,
    0U, 65U, 213U, 86U, 16U, 8U, 61U, 189U, 120U, 0U, 9U, 199U, 22U, 73U, 216U,
    253U, 204U, 2U, 176U, 50U, 75U, 106U, 12U, 227U, 60U, 2U, 0U, 0U, 38U, 141U,
    102U, 66U, 45U, 180U, 0U, 0U, 0U, 0U, 3U, 167U, 74U, 106U, 12U, 224U, 59U,
    3U, 0U, 0U, 48U, 141U, 8U, 199U, 22U, 71U, 216U, 254U, 204U, 3U, 186U, 51U,
    0U, 65U, 213U, 84U, 14U, 9U, 59U, 187U, 124U, 0U, 0U, 0U, 46U, 177U, 241U,
    246U, 198U, 81U, 0U, 0U, 156U, MAX_uint8_T, 233U, 56U, 0U, 2U, 159U, 111U,
    145U, 239U, MAX_uint8_T, 121U, 183U, 237U, 165U, 227U, 0U, 0U, 126U, 72U,
    57U, 141U, 0U, 107U, 164U, 45U, 206U, 21U, 32U, 241U, 16U, 204U, 85U, 0U, 0U,
    107U, 164U, 45U, 206U, 21U, 0U, 0U, 126U, 73U, 57U, 142U, 180U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U,
    0U, 0U, 0U, 0U, 160U, 60U, 0U, 0U, 0U, 0U, 0U, 0U, 160U, 60U, 32U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 20U, 8U,
    154U, 239U, 221U, 84U, 0U, 132U, 109U, 12U, 35U, 180U, 39U, 152U, 64U, 254U,
    167U, 61U, 91U, 133U, 170U, MAX_uint8_T, 184U, 178U, 39U, 9U, 157U, 240U,
    221U, 86U, 0U, 128U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    124U, 0U, 149U, 244U, 186U, 11U, 18U, 201U, 27U, 160U, 73U, 0U, 149U, 245U,
    186U, 11U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 52U, 0U, 0U,
    0U, 180U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 60U, 0U, 0U, 0U, 168U, 52U, 0U, 0U, 0U, 0U, 0U, 0U, 168U, 52U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 180U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, MAX_uint8_T,
    250U, 167U, 2U, 0U, 0U, 16U, 221U, 36U, 0U, 0U, 23U, 212U, 5U, 0U, 9U, 188U,
    60U, 0U, 2U, 173U, 82U, 0U, 0U, 32U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    40U, 0U, MAX_uint8_T, 249U, 160U, 0U, 0U, 0U, 44U, 215U, 0U, 0U, 152U, 253U,
    94U, 0U, 0U, 1U, 38U, 225U, 12U, 0U, 0U, 29U, 227U, 19U, 20U, MAX_uint8_T,
    244U, 131U, 0U, 0U, 163U, 151U, 0U, 85U, 166U, 2U, 0U, 216U, 80U, 0U, 0U,
    204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U,
    80U, 0U, 0U, 204U, 92U, 216U, 81U, 0U, 0U, 216U, 92U, 216U, 144U, 13U, 139U,
    245U, 92U, 216U, 231U, 244U, 122U, 204U, 92U, 216U, 80U, 0U, 0U, 0U, 0U,
    216U, 80U, 0U, 0U, 0U, 0U, 62U, 214U, 249U, MAX_uint8_T, MAX_uint8_T, 8U,
    174U, MAX_uint8_T, MAX_uint8_T, 116U, 176U, 8U, 145U, MAX_uint8_T,
    MAX_uint8_T, 116U, 176U, 8U, 18U, 169U, 243U, 116U, 176U, 8U, 0U, 0U, 72U,
    116U, 176U, 8U, 0U, 0U, 72U, 116U, 176U, 8U, 0U, 0U, 72U, 116U, 176U, 8U, 0U,
    0U, 72U, 116U, 176U, 8U, 0U, 0U, 72U, 116U, 176U, 8U, 0U, 0U, 72U, 116U,
    176U, 8U, 0U, 0U, 72U, 116U, 176U, 8U, 212U, 160U, 0U, 156U, 162U, 0U, 0U,
    18U, 226U, 0U, 4U, MAX_uint8_T, 161U, 0U, 45U, 170U, 42U, 0U, 192U, 235U,
    44U, 0U, 0U, 176U, 44U, 0U, 0U, 176U, 44U, 0U, 0U, 176U, 44U, 0U, 216U,
    MAX_uint8_T, MAX_uint8_T, 88U, 0U, 141U, 241U, 222U, 66U, 48U, 222U, 26U,
    76U, 208U, 48U, 221U, 17U, 76U, 208U, 0U, 144U, 243U, 222U, 68U, 81U, 117U,
    17U, 173U, 7U, 0U, 1U, 171U, 99U, 94U, 174U, 3U, 0U, 21U, 242U, 27U, 189U,
    100U, 1U, 171U, 100U, 94U, 174U, 3U, 81U, 117U, 17U, 174U, 7U, 0U, 19U, 133U,
    106U, 0U, 0U, 0U, 38U, 165U, 0U, 0U, 133U, 222U, 120U, 0U, 0U, 5U, 178U, 20U,
    0U, 0U, 0U, 104U, 120U, 0U, 0U, 132U, 71U, 0U, 0U, 0U, 0U, 104U, 120U, 0U,
    61U, 141U, 1U, 201U, 80U, 0U, 0U, 104U, 120U, 15U, 180U, 8U, 97U, 210U, 80U,
    0U, 0U, 104U, 120U, 157U, 46U, 14U, 171U, 124U, 80U, 0U, 0U, 0U, 90U, 112U,
    0U, 138U, 49U, 124U, 80U, 0U, 0U, 31U, 171U, 1U, 0U, 219U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 8U, 2U, 175U, 26U, 0U, 0U, 0U, 0U, 124U, 80U, 0U,
    19U, 133U, 106U, 0U, 0U, 0U, 103U, 100U, 0U, 0U, 133U, 222U, 120U, 0U, 0U,
    38U, 165U, 0U, 0U, 0U, 0U, 104U, 120U, 0U, 5U, 178U, 20U, 0U, 0U, 0U, 0U,
    104U, 120U, 0U, 134U, 106U, MAX_uint8_T, 246U, 134U, 0U, 0U, 104U, 120U, 62U,
    140U, 0U, 0U, 28U, 243U, 1U, 0U, 104U, 136U, 180U, 7U, 0U, 0U, 60U, 184U, 0U,
    0U, 0U, 158U, 44U, 0U, 0U, 24U, 205U, 28U, 0U, 0U, 92U, 111U, 0U, 0U, 9U,
    191U, 59U, 0U, 0U, 32U, 170U, 1U, 0U, 0U, 68U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 4U, 88U, MAX_uint8_T, 237U, 84U, 0U, 0U, 0U, 134U, 70U, 0U, 0U,
    3U, 124U, 131U, 0U, 0U, 62U, 140U, 0U, 0U, 0U, 240U, 229U, 30U, 0U, 16U,
    180U, 7U, 0U, 0U, 0U, 5U, 101U, 171U, 0U, 158U, 46U, 201U, 80U, 0U, 0U, 1U,
    88U, 186U, 92U, 111U, 97U, 210U, 80U, 0U, 108U, MAX_uint8_T, 226U, 95U, 170U,
    15U, 171U, 124U, 80U, 0U, 0U, 0U, 3U, 175U, 25U, 138U, 49U, 124U, 80U, 0U,
    0U, 0U, 124U, 80U, 0U, 219U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 8U, 0U,
    53U, 149U, 0U, 0U, 0U, 0U, 124U, 80U, 0U, 0U, 0U, 140U, 156U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 142U, 145U, 0U, 0U, 0U, 181U, 87U, 0U, 0U, 90U, 195U, 1U, 0U,
    67U, 236U, 29U, 0U, 0U, 177U, 144U, 0U, 0U, 0U, 164U, 185U, 18U, 0U, 0U, 28U,
    184U, 244U, MAX_uint8_T, 220U, 0U, 0U, 74U, 222U, 17U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 87U, 166U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 166U, 236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U,
    0U, 0U, 106U, 187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U,
    23U, 0U, 0U, 0U, 46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U,
    0U, 167U, 211U, 0U, 0U, 6U, 236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 51U, 0U, 85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U,
    0U, 183U, 124U, 0U, 0U, 0U, 0U, 108U, 236U, 6U, 0U, 0U, 0U, 0U, 170U, 143U,
    0U, 0U, 0U, 0U, 0U, 0U, 94U, 158U, 1U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 166U, 236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U,
    83U, 0U, 0U, 0U, 0U, 0U, 106U, 187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U,
    101U, 85U, 251U, 23U, 0U, 0U, 0U, 46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U,
    0U, 145U, 186U, 0U, 0U, 167U, 211U, 0U, 0U, 6U, 236U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 51U, 0U, 85U, 235U, 10U,
    0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U, 0U, 0U, 0U, 0U, 108U, 236U, 6U, 0U,
    0U, 6U, 206U, 229U, 45U, 0U, 0U, 0U, 0U, 0U, 138U, 109U, 45U, 193U, 7U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 166U, 236U, 6U, 0U, 0U,
    0U, 0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U, 0U, 0U, 106U, 187U, 171U, 179U,
    0U, 0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U, 23U, 0U, 0U, 0U, 46U, 251U, 20U,
    10U, 244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U, 0U, 167U, 211U, 0U, 0U, 6U,
    236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 51U,
    0U, 85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U, 0U, 0U, 0U, 0U,
    108U, 236U, 6U, 0U, 0U, 114U, 232U, 70U, 194U, 6U, 0U, 0U, 0U, 0U, 190U, 44U,
    206U, 173U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 166U,
    236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U, 0U, 0U, 106U,
    187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U, 23U, 0U, 0U, 0U,
    46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U, 0U, 167U, 211U,
    0U, 0U, 6U, 236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 51U, 0U, 85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U,
    0U, 0U, 0U, 0U, 108U, 236U, 6U, 0U, 0U, 140U, 120U, 56U, 200U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 166U, 236U, 6U, 0U, 0U, 0U,
    0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U, 0U, 0U, 106U, 187U, 171U, 179U, 0U,
    0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U, 23U, 0U, 0U, 0U, 46U, 251U, 20U, 10U,
    244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U, 0U, 167U, 211U, 0U, 0U, 6U, 236U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 51U, 0U,
    85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U, 0U, 0U, 0U, 0U, 108U,
    236U, 6U, 0U, 0U, 3U, 187U, 227U, 38U, 0U, 0U, 0U, 0U, 0U, 34U, 143U, 69U,
    113U, 0U, 0U, 0U, 0U, 0U, 3U, 187U, 228U, 39U, 0U, 0U, 0U, 0U, 0U, 0U, 166U,
    236U, 6U, 0U, 0U, 0U, 0U, 0U, 16U, 243U, 246U, 83U, 0U, 0U, 0U, 0U, 0U, 106U,
    187U, 171U, 179U, 0U, 0U, 0U, 0U, 0U, 204U, 101U, 85U, 251U, 23U, 0U, 0U, 0U,
    46U, 251U, 20U, 10U, 244U, 115U, 0U, 0U, 0U, 145U, 186U, 0U, 0U, 167U, 211U,
    0U, 0U, 6U, 236U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 51U, 0U, 85U, 235U, 10U, 0U, 0U, 4U, 224U, 147U, 0U, 183U, 124U,
    0U, 0U, 0U, 0U, 108U, 236U, 6U, 0U, 0U, 0U, 0U, 37U, 250U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 84U, 0U, 0U, 0U, 0U, 176U, 233U, 180U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 63U, 228U, 143U, 180U, 0U, 0U, 0U, 0U, 0U, 0U,
    1U, 204U, 107U, 136U, 180U, 0U, 0U, 0U, 0U, 0U, 0U, 94U, 228U, 7U, 136U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 188U, 0U, 0U, 10U, 227U, 107U, 0U,
    136U, 180U, 0U, 0U, 0U, 0U, 0U, 125U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 180U, 0U, 0U, 0U, 0U, 24U, 243U, 74U, 0U, 0U, 136U, 180U, 0U,
    0U, 0U, 0U, 156U, 149U, 0U, 0U, 0U, 136U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 152U, 0U, 0U, 98U, 207U, 248U, MAX_uint8_T,
    MAX_uint8_T, 160U, 0U, 125U, 240U, 93U, 18U, 0U, 0U, 0U, 19U, 248U, 99U, 0U,
    0U, 0U, 0U, 0U, 77U, MAX_uint8_T, 13U, 0U, 0U, 0U, 0U, 0U, 96U, 244U, 0U, 0U,
    0U, 0U, 0U, 0U, 78U, MAX_uint8_T, 15U, 0U, 0U, 0U, 0U, 0U, 21U, 250U, 108U,
    0U, 0U, 0U, 0U, 0U, 0U, 135U, 244U, 104U, 23U, 0U, 0U, 0U, 0U, 1U, 107U,
    211U, 249U, MAX_uint8_T, MAX_uint8_T, 164U, 0U, 0U, 0U, 0U, 193U, 124U, 0U,
    0U, 0U, 0U, 0U, 0U, 36U, 208U, 0U, 0U, 0U, 0U, 0U, 44U, 253U, 123U, 0U, 0U,
    9U, 194U, 109U, 0U, 0U, 0U, 0U, 14U, 196U, 42U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 248U, 0U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 96U, 0U, 224U, 88U, 0U, 0U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 56U, 0U, 0U, 71U, 218U, 24U, 0U, 0U,
    19U, 202U, 31U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, 248U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 96U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 56U, 0U, 99U, 235U, 153U, 0U, 0U, 34U, 196U, 18U, 167U, 78U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 248U,
    0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 96U, 0U, 224U, 88U, 0U,
    0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 56U, 0U, MAX_uint8_T, 4U,
    172U, 84U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 248U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 96U,
    0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U,
    0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 56U, 3U,
    173U, 137U, 0U, 0U, 6U, 183U, 64U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, 88U, 0U, 0U,
    228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U,
    228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 35U,
    228U, 50U, 3U, 189U, 61U, 0U, 0U, 0U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 228U,
    88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U,
    88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 67U,
    233U, 184U, 1U, 17U, 199U, 29U, 137U, 109U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 228U,
    88U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U,
    0U, 0U, 228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U, 0U, 0U,
    228U, 88U, 0U, 0U, 0U, 228U, 88U, 0U, 240U, 20U, 156U, 100U, 0U, 0U, 0U, 0U,
    0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U,
    0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U, 0U, 228U, 88U, 0U,
    0U, 228U, 88U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, 247U, 215U, 119U,
    2U, 0U, 0U, 0U, 224U, 88U, 2U, 19U, 87U, 239U, 135U, 0U, 0U, 0U, 224U, 88U,
    0U, 0U, 0U, 98U, 247U, 21U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 11U, MAX_uint8_T,
    68U, 48U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 76U, 0U, 0U,
    244U, 89U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 15U, MAX_uint8_T, 65U, 0U, 0U,
    224U, 88U, 0U, 0U, 0U, 102U, 240U, 10U, 0U, 0U, 224U, 88U, 0U, 23U, 93U,
    240U, 106U, 0U, 0U, 0U, 224U, MAX_uint8_T, MAX_uint8_T, 246U, 201U, 86U, 0U,
    0U, 0U, 21U, 228U, 158U, 88U, 121U, 0U, 0U, 82U, 122U, 125U, 238U, 46U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 224U, 163U, 0U, 0U, 0U, 80U, 192U, 224U,
    MAX_uint8_T, 68U, 0U, 0U, 80U, 192U, 224U, 183U, 220U, 8U, 0U, 80U, 192U,
    224U, 54U, 223U, 133U, 0U, 80U, 192U, 224U, 44U, 73U, 250U, 43U, 80U, 192U,
    224U, 44U, 0U, 168U, 196U, 81U, 192U, 224U, 44U, 0U, 23U, 239U, 183U, 192U,
    224U, 44U, 0U, 0U, 102U, MAX_uint8_T, 192U, 224U, 44U, 0U, 0U, 1U, 195U,
    192U, 0U, 0U, 6U, 185U, 121U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 10U, 191U, 51U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 101U, 213U, 249U, 231U,
    148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U, 5U, 17U, 247U, 91U,
    0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U, 0U, 0U, 0U, 183U,
    159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U, MAX_uint8_T, 12U, 0U,
    0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U, 21U, 245U, 91U, 0U, 122U,
    236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U, 215U, 250U, 230U, 148U,
    14U, 0U, 0U, 0U, 0U, 0U, 45U, 227U, 40U, 0U, 0U, 0U, 0U, 0U, 7U, 196U, 50U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 101U, 213U, 249U,
    231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U, 5U, 17U,
    247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U, 0U, 0U, 0U,
    183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U, MAX_uint8_T, 12U,
    0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U, 21U, 245U, 91U, 0U,
    122U, 236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U, 215U, 250U, 230U,
    148U, 14U, 0U, 0U, 0U, 0U, 82U, 235U, 169U, 0U, 0U, 0U, 0U, 0U, 25U, 199U,
    22U, 152U, 93U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 101U,
    213U, 249U, 231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U,
    5U, 17U, 247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U,
    0U, 0U, 0U, 183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U,
    21U, 245U, 91U, 0U, 122U, 236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U,
    215U, 250U, 230U, 148U, 14U, 0U, 0U, 0U, 8U, 218U, 175U, 69U, 145U, 0U, 0U,
    0U, 0U, 58U, 144U, 106U, 239U, 65U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 101U, 213U, 249U, 231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U,
    45U, 195U, 200U, 5U, 17U, 247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U,
    MAX_uint8_T, 11U, 0U, 0U, 0U, 0U, 183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U,
    160U, 179U, 76U, MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U,
    94U, 0U, 0U, 0U, 21U, 245U, 91U, 0U, 122U, 236U, 78U, 13U, 46U, 196U, 200U,
    5U, 0U, 0U, 103U, 215U, 250U, 230U, 148U, 14U, 0U, 0U, 0U, 0U, MAX_uint8_T,
    4U, 172U, 84U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 101U,
    213U, 249U, 231U, 148U, 14U, 0U, 0U, 122U, 235U, 80U, 14U, 45U, 195U, 200U,
    5U, 17U, 247U, 91U, 0U, 0U, 0U, 21U, 244U, 92U, 76U, MAX_uint8_T, 11U, 0U,
    0U, 0U, 0U, 183U, 159U, 96U, 243U, 0U, 0U, 0U, 0U, 0U, 160U, 179U, 76U,
    MAX_uint8_T, 12U, 0U, 0U, 0U, 0U, 184U, 159U, 17U, 246U, 94U, 0U, 0U, 0U,
    21U, 245U, 91U, 0U, 122U, 236U, 78U, 13U, 46U, 196U, 200U, 5U, 0U, 0U, 103U,
    215U, 250U, 230U, 148U, 14U, 0U, 109U, 107U, 0U, 0U, 0U, 17U, 183U, 13U, 7U,
    184U, 106U, 0U, 17U, 201U, 78U, 0U, 0U, 10U, 193U, 118U, 203U, 88U, 0U, 0U,
    0U, 0U, 28U, 249U, 160U, 0U, 0U, 0U, 0U, 10U, 193U, 118U, 204U, 88U, 0U, 0U,
    6U, 184U, 106U, 0U, 18U, 201U, 77U, 0U, 108U, 107U, 0U, 0U, 0U, 18U, 183U,
    13U, 0U, 0U, 102U, 213U, 248U, 225U, 140U, 187U, 71U, 0U, 122U, 235U, 79U,
    13U, 43U, 216U, 219U, 4U, 17U, 247U, 91U, 0U, 0U, 69U, 188U, 244U, 91U, 76U,
    MAX_uint8_T, 11U, 0U, 32U, 206U, 20U, 180U, 158U, 95U, 242U, 0U, 9U, 198U,
    51U, 0U, 159U, 179U, 75U, MAX_uint8_T, 10U, 163U, 96U, 0U, 0U, 184U, 159U,
    16U, 246U, 183U, 149U, 0U, 0U, 21U, 245U, 91U, 0U, 137U, 253U, 82U, 12U, 45U,
    195U, 200U, 5U, 17U, 206U, 114U, 208U, 246U, 233U, 150U, 15U, 0U, 0U, 56U,
    227U, 29U, 0U, 0U, 0U, 0U, 0U, 68U, 183U, 2U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U,
    72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U,
    0U, 0U, 208U, 68U, 240U, 75U, 0U, 0U, 0U, 210U, 64U, 214U, 105U, 0U, 0U, 1U,
    235U, 39U, 137U, 216U, 43U, 11U, 110U, 221U, 4U, 12U, 150U, 229U, 243U, 197U,
    51U, 0U, 0U, 0U, 0U, 147U, 165U, 2U, 0U, 0U, 0U, 70U, 178U, 4U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U,
    0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U,
    68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 240U, 75U, 0U, 0U, 0U, 210U, 64U,
    214U, 105U, 0U, 0U, 1U, 235U, 39U, 137U, 216U, 43U, 11U, 110U, 221U, 4U, 12U,
    150U, 229U, 243U, 197U, 51U, 0U, 0U, 2U, 191U, 232U, 60U, 0U, 0U, 0U, 118U,
    129U, 34U, 198U, 14U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 244U, 72U, 0U, 0U, 0U,
    208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U,
    68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U,
    240U, 75U, 0U, 0U, 0U, 210U, 64U, 214U, 105U, 0U, 0U, 1U, 235U, 39U, 137U,
    216U, 43U, 11U, 110U, 221U, 4U, 12U, 150U, 229U, 243U, 197U, 51U, 0U, 0U,
    132U, 128U, 48U, 208U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 244U, 72U, 0U, 0U,
    0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U,
    68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U, 244U, 72U, 0U, 0U, 0U, 208U, 68U,
    240U, 75U, 0U, 0U, 0U, 210U, 64U, 214U, 105U, 0U, 0U, 1U, 235U, 39U, 137U,
    216U, 43U, 11U, 110U, 221U, 4U, 12U, 150U, 229U, 243U, 197U, 51U, 0U, 0U, 0U,
    0U, 30U, 227U, 55U, 0U, 0U, 0U, 0U, 2U, 184U, 67U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 159U, 199U, 1U, 0U, 0U, 39U, 231U, 22U, 24U, 242U, 92U,
    0U, 0U, 190U, 102U, 0U, 0U, 120U, 228U, 11U, 96U, 198U, 1U, 0U, 0U, 7U, 221U,
    150U, 231U, 46U, 0U, 0U, 0U, 0U, 81U, MAX_uint8_T, 139U, 0U, 0U, 0U, 0U, 0U,
    0U, MAX_uint8_T, 56U, 0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 56U, 0U, 0U, 0U,
    0U, 0U, 0U, MAX_uint8_T, 56U, 0U, 0U, 0U, 0U, 0U, 0U, MAX_uint8_T, 56U, 0U,
    0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U,
    MAX_uint8_T, 252U, 229U, 141U, 1U, 224U, 88U, 8U, 73U, 253U, 75U, 224U, 88U,
    0U, 0U, 238U, 89U, 224U, 88U, 16U, 126U, 239U, 26U, 224U, MAX_uint8_T, 236U,
    184U, 52U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 224U, 88U, 0U, 0U, 0U, 0U, 60U,
    219U, 246U, 185U, 17U, 0U, 182U, 133U, 20U, 218U, 96U, 0U, 212U, 80U, 12U,
    229U, 39U, 0U, 216U, 80U, 136U, 140U, 0U, 0U, 216U, 80U, 172U, 105U, 0U, 0U,
    216U, 80U, 20U, 165U, 142U, 11U, 216U, 80U, 0U, 0U, 122U, 164U, 216U, 80U,
    0U, 1U, 121U, 193U, 216U, 80U, 224U, MAX_uint8_T, 220U, 62U, 0U, 36U, 226U,
    50U, 0U, 0U, 0U, 0U, 0U, 45U, 197U, 9U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U, 149U, 44U, 23U, 229U, 87U, 0U, 0U, 0U,
    0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U, 242U, MAX_uint8_T, 108U, 0U, 52U,
    242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U, 27U, 47U, 221U, 137U, 0U, 8U, 182U,
    241U, 166U, 92U, 235U, 60U, 0U, 0U, 0U, 115U, 191U, 8U, 0U, 0U, 0U, 45U,
    195U, 13U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U,
    0U, 0U, 149U, 44U, 23U, 229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U,
    91U, 205U, 242U, MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U,
    84U, 226U, 27U, 47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U,
    0U, 0U, 166U, 235U, 85U, 0U, 0U, 0U, 89U, 156U, 21U, 198U, 27U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U, 149U, 44U, 23U,
    229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U, 242U,
    MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U, 27U,
    47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U, 0U, 67U, 239U,
    107U, 145U, 57U, 0U, 0U, 146U, 69U, 174U, 217U, 7U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U, 149U, 44U, 23U, 229U, 87U, 0U,
    0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U, 242U, MAX_uint8_T, 108U, 0U,
    52U, 242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U, 27U, 47U, 221U, 137U, 0U, 8U,
    182U, 241U, 166U, 92U, 235U, 60U, 0U, 88U, 172U, 4U, 252U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U, 149U, 44U, 23U,
    229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U, 242U,
    MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U, 27U,
    47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U, 0U, 0U, 148U,
    238U, 69U, 0U, 0U, 0U, 0U, 172U, 39U, 148U, 0U, 0U, 0U, 0U, 148U, 239U, 71U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 60U, 200U, 244U, 183U, 9U, 0U, 0U,
    149U, 44U, 23U, 229U, 87U, 0U, 0U, 0U, 0U, 0U, 188U, 107U, 0U, 0U, 91U, 205U,
    242U, MAX_uint8_T, 108U, 0U, 52U, 242U, 73U, 6U, 188U, 108U, 0U, 84U, 226U,
    27U, 47U, 221U, 137U, 0U, 8U, 182U, 241U, 166U, 92U, 235U, 60U, 0U, 60U,
    200U, 240U, 163U, 116U, 232U, 220U, 70U, 0U, 0U, 149U, 44U, 23U, 229U, 226U,
    31U, 71U, 233U, 4U, 0U, 0U, 0U, 0U, 188U, 141U, 0U, 0U, 232U, 56U, 0U, 91U,
    205U, 242U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    80U, 52U, 242U, 73U, 6U, 188U, 136U, 0U, 0U, 0U, 0U, 84U, 226U, 27U, 54U,
    201U, 238U, 64U, 5U, 0U, 0U, 8U, 182U, 246U, 176U, 21U, 115U, 229U,
    MAX_uint8_T, MAX_uint8_T, 92U, 0U, 33U, 180U, 240U, MAX_uint8_T, 128U, 3U,
    212U, 180U, 23U, 0U, 0U, 53U, MAX_uint8_T, 32U, 0U, 0U, 0U, 78U, 251U, 0U,
    0U, 0U, 0U, 46U, MAX_uint8_T, 33U, 0U, 0U, 0U, 0U, 198U, 183U, 26U, 0U, 0U,
    0U, 25U, 178U, 245U, MAX_uint8_T, 148U, 0U, 0U, 1U, 199U, 117U, 0U, 0U, 0U,
    0U, 43U, 202U, 0U, 0U, 0U, 52U, 252U, 115U, 0U, 0U, 41U, 227U, 45U, 0U, 0U,
    0U, 0U, 51U, 195U, 7U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 194U, 241U, 167U,
    9U, 2U, 211U, 108U, 15U, 183U, 115U, 52U, 244U, 2U, 0U, 93U, 186U, 78U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 211U, 47U, 241U, 4U, 0U,
    0U, 0U, 0U, 196U, 152U, 21U, 0U, 0U, 0U, 22U, 170U, 239U, MAX_uint8_T, 208U,
    0U, 0U, 0U, 123U, 185U, 6U, 0U, 0U, 51U, 191U, 10U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 35U, 194U, 241U, 167U, 9U, 2U, 211U, 108U, 15U, 183U, 115U, 52U,
    244U, 2U, 0U, 93U, 186U, 78U, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, 211U, 47U, 241U, 4U, 0U, 0U, 0U, 0U, 196U, 152U, 21U, 0U, 0U,
    0U, 22U, 170U, 239U, MAX_uint8_T, 208U, 0U, 0U, 147U, 235U, 105U, 0U, 0U,
    70U, 173U, 16U, 193U, 39U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 194U, 241U, 167U,
    9U, 2U, 211U, 108U, 15U, 183U, 115U, 52U, 244U, 2U, 0U, 93U, 186U, 78U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 211U, 47U, 241U, 4U, 0U,
    0U, 0U, 0U, 196U, 152U, 21U, 0U, 0U, 0U, 22U, 170U, 239U, MAX_uint8_T, 208U,
    0U, 52U, 208U, 0U, 224U, 32U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 194U, 241U,
    167U, 9U, 2U, 211U, 108U, 15U, 183U, 115U, 52U, 244U, 2U, 0U, 93U, 186U, 78U,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 211U, 47U, 241U, 4U, 0U,
    0U, 0U, 0U, 196U, 152U, 21U, 0U, 0U, 0U, 22U, 170U, 239U, MAX_uint8_T, 208U,
    3U, 173U, 137U, 0U, 0U, 6U, 183U, 64U, 0U, 0U, 0U, 0U, 0U, 0U, 216U, 80U, 0U,
    0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U,
    0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 35U, 228U, 50U, 3U, 189U, 61U, 0U, 0U,
    0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U,
    216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U,
    67U, 233U, 184U, 1U, 17U, 199U, 29U, 137U, 109U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    216U, 80U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 216U,
    80U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 216U, 80U, 0U,
    240U, 20U, 156U, 100U, 0U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U,
    0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U, 0U, 216U, 80U, 0U,
    0U, 216U, 80U, 0U, 0U, 1U, 5U, 139U, 56U, 0U, 0U, 90U, 227U, 251U, 226U, 21U,
    0U, 0U, 0U, 86U, 114U, 105U, 219U, 25U, 0U, 0U, 36U, 190U, 243U, 248U, 190U,
    0U, 3U, 214U, 144U, 15U, 70U, 250U, 54U, 57U, 250U, 10U, 0U, 0U, 181U, 117U,
    83U, 231U, 0U, 0U, 0U, 159U, 139U, 54U, 251U, 12U, 0U, 0U, 194U, 109U, 2U,
    210U, 147U, 15U, 89U, 243U, 24U, 0U, 33U, 187U, 242U, 208U, 63U, 0U, 0U,
    168U, 211U, 45U, 192U, 0U, 2U, 198U, 64U, 229U, 119U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 216U, 103U, 184U, 245U, 171U, 2U, 216U, 222U, 66U, 21U, 239U, 67U, 216U,
    95U, 0U, 0U, 206U, 91U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U,
    204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 216U, 80U, 0U, 0U, 204U, 92U, 0U,
    5U, 182U, 125U, 0U, 0U, 0U, 0U, 0U, 9U, 189U, 54U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 35U, 186U, 242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U,
    242U, 55U, 52U, 249U, 8U, 0U, 0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U,
    174U, 51U, 248U, 7U, 0U, 0U, 163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U,
    0U, 33U, 187U, 243U, 215U, 88U, 0U, 0U, 0U, 0U, 42U, 228U, 43U, 0U, 0U, 0U,
    6U, 194U, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 186U, 242U, 216U,
    91U, 0U, 3U, 211U, 136U, 14U, 62U, 242U, 55U, 52U, 249U, 8U, 0U, 0U, 161U,
    148U, 79U, 230U, 0U, 0U, 0U, 135U, 174U, 51U, 248U, 7U, 0U, 0U, 163U, 146U,
    2U, 210U, 134U, 13U, 64U, 243U, 53U, 0U, 33U, 187U, 243U, 215U, 88U, 0U, 0U,
    0U, 78U, 234U, 173U, 0U, 0U, 0U, 23U, 199U, 24U, 149U, 97U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 35U, 186U, 242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U,
    242U, 55U, 52U, 249U, 8U, 0U, 0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U,
    174U, 51U, 248U, 7U, 0U, 0U, 163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U,
    0U, 33U, 187U, 243U, 215U, 88U, 0U, 0U, 7U, 216U, 178U, 66U, 149U, 0U, 0U,
    54U, 148U, 103U, 239U, 68U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 35U, 186U,
    242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U, 242U, 55U, 52U, 249U, 8U, 0U,
    0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U, 174U, 51U, 248U, 7U, 0U, 0U,
    163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U, 0U, 33U, 187U, 243U, 215U,
    88U, 0U, 0U, 0U, 252U, 8U, 168U, 88U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    35U, 186U, 242U, 216U, 91U, 0U, 3U, 211U, 136U, 14U, 62U, 242U, 55U, 52U,
    249U, 8U, 0U, 0U, 161U, 148U, 79U, 230U, 0U, 0U, 0U, 135U, 174U, 51U, 248U,
    7U, 0U, 0U, 163U, 146U, 2U, 210U, 134U, 13U, 64U, 243U, 53U, 0U, 33U, 187U,
    243U, 215U, 88U, 0U, 0U, 0U, 0U, 244U, 128U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 180U, MAX_uint8_T, MAX_uint8_T,
    MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, MAX_uint8_T, 60U, 0U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 244U, 128U, 0U, 0U,
    0U, 0U, 33U, 185U, 242U, 221U, 179U, 89U, 2U, 210U, 135U, 14U, 114U,
    MAX_uint8_T, 53U, 51U, 248U, 7U, 21U, 188U, 185U, 147U, 79U, 229U, 2U, 176U,
    43U, 137U, 175U, 54U, 249U, 141U, 90U, 0U, 161U, 147U, 2U, 213U, 201U, 12U,
    62U, 242U, 55U, 20U, 191U, 190U, 245U, 217U, 91U, 0U, 5U, 182U, 125U, 0U, 0U,
    0U, 0U, 9U, 189U, 54U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 236U, 60U, 0U, 0U,
    220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U,
    60U, 0U, 0U, 220U, 72U, 235U, 61U, 0U, 4U, 230U, 72U, 212U, 112U, 21U, 162U,
    246U, 72U, 80U, 234U, 227U, 88U, 220U, 72U, 0U, 0U, 42U, 228U, 43U, 0U, 0U,
    6U, 194U, 53U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 236U, 60U, 0U, 0U, 220U, 72U,
    236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U,
    0U, 220U, 72U, 235U, 61U, 0U, 4U, 230U, 72U, 212U, 112U, 21U, 162U, 246U,
    72U, 80U, 234U, 227U, 88U, 220U, 72U, 0U, 78U, 234U, 173U, 0U, 0U, 23U, 199U,
    24U, 149U, 97U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 236U, 60U, 0U, 0U, 220U, 72U,
    236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U,
    0U, 220U, 72U, 235U, 61U, 0U, 4U, 230U, 72U, 212U, 112U, 21U, 162U, 246U,
    72U, 80U, 234U, 227U, 88U, 220U, 72U, 0U, 252U, 8U, 168U, 88U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U,
    236U, 60U, 0U, 0U, 220U, 72U, 236U, 60U, 0U, 0U, 220U, 72U, 235U, 61U, 0U,
    4U, 230U, 72U, 212U, 112U, 21U, 162U, 246U, 72U, 80U, 234U, 227U, 88U, 220U,
    72U, 0U, 0U, 0U, 166U, 147U, 0U, 0U, 0U, 0U, 89U, 162U, 1U, 0U, 0U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 187U, 131U, 0U, 0U, 23U, 240U, 13U, 97U, 220U, 0U, 0U,
    114U, 158U, 0U, 14U, 247U, 53U, 0U, 208U, 59U, 0U, 0U, 172U, 142U, 45U, 216U,
    0U, 0U, 0U, 82U, 229U, 142U, 117U, 0U, 0U, 0U, 7U, 240U, 244U, 23U, 0U, 0U,
    0U, 0U, 157U, 175U, 0U, 0U, 0U, 0U, 0U, 179U, 76U, 0U, 0U, 0U, 0U, 54U, 229U,
    3U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 216U,
    90U, 179U, 242U, 173U, 14U, 216U, 196U, 66U, 22U, 206U, 137U, 216U, 105U, 0U,
    0U, 108U, 207U, 216U, 80U, 0U, 0U, 90U, 222U, 216U, 104U, 0U, 0U, 120U, 191U,
    216U, 209U, 53U, 34U, 220U, 98U, 216U, 99U, 198U, 244U, 147U, 2U, 216U, 80U,
    0U, 0U, 0U, 0U, 216U, 80U, 0U, 0U, 0U, 0U, 0U, 116U, 144U, 32U, 224U, 0U, 0U,
    0U, 0U, 0U, 0U, 0U, 0U, 0U, 187U, 131U, 0U, 0U, 23U, 240U, 13U, 97U, 220U,
    0U, 0U, 114U, 158U, 0U, 14U, 247U, 53U, 0U, 208U, 59U, 0U, 0U, 172U, 142U,
    45U, 216U, 0U, 0U, 0U, 82U, 229U, 142U, 117U, 0U, 0U, 0U, 7U, 240U, 244U,
    23U, 0U, 0U, 0U, 0U, 157U, 175U, 0U, 0U, 0U, 0U, 0U, 179U, 76U, 0U, 0U, 0U,
    0U, 54U, 229U, 3U, 0U, 0U, 0U };

  static boolean_T c4_bv[2] = { false, true };

  void* c4_b_colPtr;
  void* c4_b_posPtr;
  void* c4_b_ptrObj;
  void* c4_c_colPtr;
  void* c4_c_posPtr;
  void* c4_c_ptrObj;
  void* c4_colPtr;
  void* c4_d_ptrObj;
  void* c4_e_ptrObj;
  void* c4_posPtr;
  void* c4_ptrObj;
  c4_emxArray_cell_wrap_21_28 c4_pos;
  dim3 c4_ab_block;
  dim3 c4_ab_grid;
  dim3 c4_b_block;
  dim3 c4_b_grid;
  dim3 c4_bb_block;
  dim3 c4_bb_grid;
  dim3 c4_block;
  dim3 c4_c_block;
  dim3 c4_c_grid;
  dim3 c4_cb_block;
  dim3 c4_cb_grid;
  dim3 c4_d_block;
  dim3 c4_d_grid;
  dim3 c4_db_block;
  dim3 c4_db_grid;
  dim3 c4_e_block;
  dim3 c4_e_grid;
  dim3 c4_eb_block;
  dim3 c4_eb_grid;
  dim3 c4_f_block;
  dim3 c4_f_grid;
  dim3 c4_fb_block;
  dim3 c4_fb_grid;
  dim3 c4_g_block;
  dim3 c4_g_grid;
  dim3 c4_gb_block;
  dim3 c4_gb_grid;
  dim3 c4_grid;
  dim3 c4_h_block;
  dim3 c4_h_grid;
  dim3 c4_hb_block;
  dim3 c4_hb_grid;
  dim3 c4_i_block;
  dim3 c4_i_grid;
  dim3 c4_ib_block;
  dim3 c4_ib_grid;
  dim3 c4_j_block;
  dim3 c4_j_grid;
  dim3 c4_jb_block;
  dim3 c4_jb_grid;
  dim3 c4_k_block;
  dim3 c4_k_grid;
  dim3 c4_kb_block;
  dim3 c4_kb_grid;
  dim3 c4_l_block;
  dim3 c4_l_grid;
  dim3 c4_lb_block;
  dim3 c4_lb_grid;
  dim3 c4_m_block;
  dim3 c4_m_grid;
  dim3 c4_mb_block;
  dim3 c4_mb_grid;
  dim3 c4_n_block;
  dim3 c4_n_grid;
  dim3 c4_o_block;
  dim3 c4_o_grid;
  dim3 c4_p_block;
  dim3 c4_p_grid;
  dim3 c4_q_block;
  dim3 c4_q_grid;
  dim3 c4_r_block;
  dim3 c4_r_grid;
  dim3 c4_s_block;
  dim3 c4_s_grid;
  dim3 c4_t_block;
  dim3 c4_t_grid;
  dim3 c4_u_block;
  dim3 c4_u_grid;
  dim3 c4_v_block;
  dim3 c4_v_grid;
  dim3 c4_w_block;
  dim3 c4_w_grid;
  dim3 c4_x_block;
  dim3 c4_x_grid;
  dim3 c4_y_block;
  dim3 c4_y_grid;
  real_T c4_thisGlyphCut_float_data[132];
  real_T c4_cg;
  real_T c4_d2;
  real_T c4_d3;
  real_T c4_d4;
  real_T c4_d5;
  real_T c4_d6;
  real_T c4_d7;
  real_T c4_d8;
  real_T c4_glyphVal;
  real_T c4_lenEndSegment;
  real_T c4_lenFirstSegment;
  real_T c4_lenThisSegment;
  real_T c4_rg;
  real_T c4_y;
  int32_T c4_b_positionOut_data[112];
  int32_T c4_positionOut_data[112];
  int32_T c4_position[56];
  int32_T c4_textPosition_data[40];
  int32_T c4_shapeHeight_data[20];
  int32_T c4_shapeWidth_data[20];
  int32_T c4_varargin_1[3];
  int32_T c4_b_color_size[2];
  int32_T c4_color_size[2];
  int32_T c4_idxNewlineChar_size[2];
  int32_T c4_ii_size[2];
  int32_T c4_isNewLineChar_size[2];
  int32_T c4_iv[2];
  int32_T c4_iv1[2];
  int32_T c4_position_size[2];
  int32_T c4_textColor_size[2];
  int32_T c4_textLocAndWidth_size[2];
  int32_T c4_textPosition_size[2];
  int32_T c4_thisCharcodes_1b_size[2];
  int32_T c4_thisGlyphCut_float_size[2];
  int32_T c4_thisTextU16_size[2];
  int32_T c4_uv4_size[2];
  int32_T c4_x_size[2];
  int32_T c4_b_textLocAndWidth_size[1];
  int32_T c4_ab_qY;
  int32_T c4_b_c;
  int32_T c4_b_i;
  int32_T c4_b_ibmat;
  int32_T c4_b_idx;
  int32_T c4_b_ii;
  int32_T c4_b_itilerow;
  int32_T c4_b_jcol;
  int32_T c4_b_k;
  int32_T c4_b_ntilerows;
  int32_T c4_b_nz;
  int32_T c4_b_q0;
  int32_T c4_b_q1;
  int32_T c4_b_qY;
  int32_T c4_b_r;
  int32_T c4_b_tbWidth;
  int32_T c4_b_vlen;
  int32_T c4_bb_qY;
  int32_T c4_bitmapEndIdx_1b;
  int32_T c4_c;
  int32_T c4_c_c;
  int32_T c4_c_i;
  int32_T c4_c_ibmat;
  int32_T c4_c_itilerow;
  int32_T c4_c_jcol;
  int32_T c4_c_ntilerows;
  int32_T c4_c_nz;
  int32_T c4_c_q0;
  int32_T c4_c_q1;
  int32_T c4_c_qY;
  int32_T c4_c_r;
  int32_T c4_c_vlen;
  int32_T c4_d_c;
  int32_T c4_d_i;
  int32_T c4_d_nz;
  int32_T c4_d_q0;
  int32_T c4_d_q1;
  int32_T c4_d_qY;
  int32_T c4_d_r;
  int32_T c4_d_vlen;
  int32_T c4_e_i;
  int32_T c4_e_q0;
  int32_T c4_e_q1;
  int32_T c4_e_qY;
  int32_T c4_e_vlen;
  int32_T c4_endC;
  int32_T c4_endC_gl;
  int32_T c4_endC_im;
  int32_T c4_endIdx;
  int32_T c4_endR;
  int32_T c4_endR_gl;
  int32_T c4_endR_im;
  int32_T c4_f_i;
  int32_T c4_f_q0;
  int32_T c4_f_q1;
  int32_T c4_f_qY;
  int32_T c4_f_vlen;
  int32_T c4_g_i;
  int32_T c4_g_q0;
  int32_T c4_g_q1;
  int32_T c4_g_qY;
  int32_T c4_g_vlen;
  int32_T c4_h_i;
  int32_T c4_h_q1;
  int32_T c4_h_qY;
  int32_T c4_i;
  int32_T c4_i1;
  int32_T c4_i10;
  int32_T c4_i102;
  int32_T c4_i11;
  int32_T c4_i12;
  int32_T c4_i127;
  int32_T c4_i13;
  int32_T c4_i14;
  int32_T c4_i15;
  int32_T c4_i25;
  int32_T c4_i28;
  int32_T c4_i3;
  int32_T c4_i34;
  int32_T c4_i4;
  int32_T c4_i5;
  int32_T c4_i6;
  int32_T c4_i7;
  int32_T c4_i73;
  int32_T c4_i75;
  int32_T c4_i8;
  int32_T c4_i87;
  int32_T c4_i88;
  int32_T c4_i9;
  int32_T c4_i90;
  int32_T c4_i91;
  int32_T c4_i93;
  int32_T c4_i95;
  int32_T c4_i_i;
  int32_T c4_i_q1;
  int32_T c4_i_qY;
  int32_T c4_ibmat;
  int32_T c4_idx;
  int32_T c4_ii;
  int32_T c4_itilerow;
  int32_T c4_j_i;
  int32_T c4_j_qY;
  int32_T c4_jcol;
  int32_T c4_k;
  int32_T c4_k_i;
  int32_T c4_k_qY;
  int32_T c4_l_qY;
  int32_T c4_m_qY;
  int32_T c4_maxLen;
  int32_T c4_n_qY;
  int32_T c4_ntilerows;
  int32_T c4_nx;
  int32_T c4_nz;
  int32_T c4_o_qY;
  int32_T c4_p_qY;
  int32_T c4_penX;
  int32_T c4_penY;
  int32_T c4_q0;
  int32_T c4_q1;
  int32_T c4_qY;
  int32_T c4_q_qY;
  int32_T c4_r;
  int32_T c4_r_qY;
  int32_T c4_s_qY;
  int32_T c4_startC;
  int32_T c4_startC_gl;
  int32_T c4_startC_im;
  int32_T c4_startR;
  int32_T c4_startR_gl;
  int32_T c4_startR_im;
  int32_T c4_t_qY;
  int32_T c4_tbHeight;
  int32_T c4_tbTopLeftX;
  int32_T c4_tbTopLeftY;
  int32_T c4_tbWidth;
  int32_T c4_textIdx;
  int32_T c4_u_qY;
  int32_T c4_v_qY;
  int32_T c4_vlen;
  int32_T c4_w_qY;
  int32_T c4_x_qY;
  int32_T c4_y_qY;
  int32_T c4_yy;
  real32_T c4_b_color[84];
  real32_T c4_c_color_data[60];
  real32_T c4_prevpt[2];
  real32_T c4_tmp11;
  real32_T c4_tmp22;
  int16_T c4_dimens1;
  int16_T c4_dimens2;
  int16_T c4_numFillColor;
  char_T c4_str1[30];
  char_T c4_cv7[6];
  int8_T c4_b_color_data[60];
  int8_T c4_color_data[60];
  int8_T c4_textColor_data[60];
  int8_T c4_idxNewlineChar_data[29];
  int8_T c4_ii_data[29];
  int8_T c4_outsize[2];
  int8_T c4_outsize_idx_0;
  int8_T c4_outsize_idx_1;
  uint8_T c4_pixCount[640];
  uint8_T c4_thisTextU16_data[29];
  boolean_T c4_isNewLineChar_data[29];
  boolean_T c4_I_dirtyOnGpu;
  boolean_T c4_In_dirtyOnCpu;
  boolean_T c4_In_dirtyOnGpu;
  boolean_T c4_ab_validLaunchParams;
  boolean_T c4_b_color_data_dirtyOnCpu;
  boolean_T c4_b_isInitialise;
  boolean_T c4_b_nz_dirtyOnCpu;
  boolean_T c4_b_nz_dirtyOnGpu;
  boolean_T c4_b_positionOut_data_dirtyOnGpu;
  boolean_T c4_b_validLaunchParams;
  boolean_T c4_bb_validLaunchParams;
  boolean_T c4_c_isInitialise;
  boolean_T c4_c_nz_dirtyOnCpu;
  boolean_T c4_c_validLaunchParams;
  boolean_T c4_cb_validLaunchParams;
  boolean_T c4_color_data_dirtyOnCpu;
  boolean_T c4_color_data_dirtyOnGpu;
  boolean_T c4_color_dirtyOnCpu;
  boolean_T c4_color_dirtyOnGpu;
  boolean_T c4_color_size_dirtyOnCpu;
  boolean_T c4_cv8_dirtyOnCpu;
  boolean_T c4_d_isInitialise;
  boolean_T c4_d_validLaunchParams;
  boolean_T c4_db_validLaunchParams;
  boolean_T c4_e_isInitialise;
  boolean_T c4_e_validLaunchParams;
  boolean_T c4_eb_validLaunchParams;
  boolean_T c4_exitg1;
  boolean_T c4_f_validLaunchParams;
  boolean_T c4_fb_validLaunchParams;
  boolean_T c4_fv1_dirtyOnCpu;
  boolean_T c4_g_validLaunchParams;
  boolean_T c4_gb_validLaunchParams;
  boolean_T c4_guard1 = false;
  boolean_T c4_h_validLaunchParams;
  boolean_T c4_hb_validLaunchParams;
  boolean_T c4_i_validLaunchParams;
  boolean_T c4_ib_validLaunchParams;
  boolean_T c4_idxNewlineChar_data_dirtyOnGpu;
  boolean_T c4_ii_data_dirtyOnCpu;
  boolean_T c4_isInitialise;
  boolean_T c4_isNewLineChar_data_dirtyOnGpu;
  boolean_T c4_isScalarText;
  boolean_T c4_iv1_dirtyOnCpu;
  boolean_T c4_j_validLaunchParams;
  boolean_T c4_jb_validLaunchParams;
  boolean_T c4_k_validLaunchParams;
  boolean_T c4_kb_validLaunchParams;
  boolean_T c4_l_validLaunchParams;
  boolean_T c4_lb_validLaunchParams;
  boolean_T c4_lenEndSegment_dirtyOnCpu;
  boolean_T c4_lenEndSegment_dirtyOnGpu;
  boolean_T c4_lenFirstSegment_dirtyOnCpu;
  boolean_T c4_lenFirstSegment_dirtyOnGpu;
  boolean_T c4_lenThisSegment_dirtyOnCpu;
  boolean_T c4_lenThisSegment_dirtyOnGpu;
  boolean_T c4_ltPts_dirtyOnCpu;
  boolean_T c4_m_validLaunchParams;
  boolean_T c4_mb_validLaunchParams;
  boolean_T c4_n_validLaunchParams;
  boolean_T c4_nz_dirtyOnCpu;
  boolean_T c4_nz_dirtyOnGpu;
  boolean_T c4_o_validLaunchParams;
  boolean_T c4_p_validLaunchParams;
  boolean_T c4_pixCount_dirtyOnGpu;
  boolean_T c4_positionOut_data_dirtyOnGpu;
  boolean_T c4_position_dirtyOnGpu;
  boolean_T c4_position_size_dirtyOnCpu;
  boolean_T c4_q1_dirtyOnGpu;
  boolean_T c4_q_validLaunchParams;
  boolean_T c4_r_validLaunchParams;
  boolean_T c4_rtPts_dirtyOnCpu;
  boolean_T c4_s_validLaunchParams;
  boolean_T c4_shapeHeight_data_dirtyOnGpu;
  boolean_T c4_shapeWidth_data_dirtyOnGpu;
  boolean_T c4_t_validLaunchParams;
  boolean_T c4_textColor_data_dirtyOnGpu;
  boolean_T c4_textLocAndWidth_size_dirtyOnCpu;
  boolean_T c4_textPosition_data_dirtyOnGpu;
  boolean_T c4_thisCharcodes_1b_size_dirtyOnCpu;
  boolean_T c4_thisGlyphCut_float_data_dirtyOnGpu;
  boolean_T c4_thisTextU16_data_dirtyOnGpu;
  boolean_T c4_thisTextU16_size_dirtyOnCpu;
  boolean_T c4_u_validLaunchParams;
  boolean_T c4_uv4_dirtyOnCpu;
  boolean_T c4_uv_dirtyOnCpu;
  boolean_T c4_v_validLaunchParams;
  boolean_T c4_validLaunchParams;
  boolean_T c4_w_validLaunchParams;
  boolean_T c4_x_validLaunchParams;
  boolean_T c4_y_validLaunchParams;
  c4_nz_dirtyOnCpu = false;
  c4_lenEndSegment_dirtyOnCpu = false;
  c4_b_nz_dirtyOnCpu = false;
  c4_lenThisSegment_dirtyOnCpu = false;
  c4_c_nz_dirtyOnCpu = false;
  c4_lenFirstSegment_dirtyOnCpu = false;
  c4_ii_data_dirtyOnCpu = false;
  c4_color_data_dirtyOnCpu = false;
  c4_b_color_data_dirtyOnCpu = false;
  c4_thisGlyphCut_float_data_dirtyOnGpu = false;
  c4_idxNewlineChar_data_dirtyOnGpu = false;
  c4_isNewLineChar_data_dirtyOnGpu = false;
  c4_thisTextU16_data_dirtyOnGpu = false;
  c4_shapeHeight_data_dirtyOnGpu = false;
  c4_shapeWidth_data_dirtyOnGpu = false;
  c4_textColor_data_dirtyOnGpu = false;
  c4_textPosition_data_dirtyOnGpu = false;
  c4_color_data_dirtyOnGpu = false;
  c4_positionOut_data_dirtyOnGpu = false;
  c4_uv4_dirtyOnCpu = true;
  c4_uv_dirtyOnCpu = true;
  c4_iv1_dirtyOnCpu = true;
  c4_cv8_dirtyOnCpu = true;
  c4_In_dirtyOnGpu = false;
  c4_In_dirtyOnCpu = true;
  c4_rtPts_dirtyOnCpu = true;
  c4_ltPts_dirtyOnCpu = true;
  if ((c4_bboxes_size[0] != 0) && (c4_bboxes_size[1] != 0)) {
    c4_position_size[0] = c4_bboxes_size[0];
    c4_position_size[1] = c4_bboxes_size[1];
    c4_position_size_dirtyOnCpu = true;
    c4_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c4_bboxes_size[0] * c4_bboxes_size[1] - 1) + 1L), &c4_grid, &c4_block,
      1024U, 65535U);
    if (c4_validLaunchParams) {
      cudaMemcpy(chartInstance->c4_gpu_bboxes_data, (void *)c4_b_bboxes_data,
                 (uint32_T)(c4_bboxes_size[0] * c4_bboxes_size[1]) * sizeof
                 (real_T), cudaMemcpyHostToDevice);
      cudaMemcpy(chartInstance->c4_gpu_bboxes_size, &c4_bboxes_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      c4_eML_blk_kernel_kernel2<<<c4_grid, c4_block>>>
        (chartInstance->c4_gpu_bboxes_data, *chartInstance->c4_gpu_bboxes_size, *
         chartInstance->c4_gpu_position_data);
    }

    c4_color_size[0] = (int8_T)c4_position_size[0];
    c4_color_size[1] = 3;
    c4_color_size_dirtyOnCpu = true;
    c4_ntilerows = (int8_T)c4_position_size[0];
    for (c4_jcol = 0; c4_jcol < 3; c4_jcol++) {
      c4_ibmat = c4_jcol * c4_ntilerows;
      for (c4_itilerow = 0; c4_itilerow < c4_ntilerows; c4_itilerow++) {
        c4_color_data[c4_ibmat + c4_itilerow] = c4_b_fv[c4_jcol];
        c4_b_color_data_dirtyOnCpu = true;
      }
    }

    if ((int8_T)c4_position_size[0] == 1) {
      c4_b_color_size[0] = (int8_T)c4_position_size[0];
      c4_b_color_size[1] = 3;
      c4_b_ntilerows = (int8_T)c4_position_size[0];
      for (c4_b_jcol = 0; c4_b_jcol < 3; c4_b_jcol++) {
        c4_b_ibmat = c4_b_jcol * c4_b_ntilerows;
        for (c4_b_itilerow = 0; c4_b_itilerow < c4_b_ntilerows; c4_b_itilerow++)
        {
          c4_b_color_data[c4_b_ibmat + c4_b_itilerow] = c4_color_data[c4_b_jcol];
          c4_color_data_dirtyOnCpu = true;
        }
      }
    } else {
      c4_b_color_size[0] = (int8_T)c4_position_size[0];
      c4_b_color_size[1] = 3;
      c4_b_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
        (c4_color_size[0] * 3 - 1) + 1L), &c4_b_grid, &c4_b_block, 1024U, 65535U);
      if (c4_b_validLaunchParams) {
        if (c4_b_color_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c4_c_gpu_color_data, &c4_color_data[0], 60UL,
                     cudaMemcpyHostToDevice);
          c4_b_color_data_dirtyOnCpu = false;
        }

        cudaMemcpy(chartInstance->c4_b_gpu_color_size, &c4_color_size[0], 8UL,
                   cudaMemcpyHostToDevice);
        c4_color_size_dirtyOnCpu = false;
        c4_eML_blk_kernel_kernel3<<<c4_b_grid, c4_b_block>>>
          (*chartInstance->c4_c_gpu_color_data,
           *chartInstance->c4_b_gpu_color_size,
           *chartInstance->c4_gpu_color_data);
      }
    }

    cudaMemcpy(chartInstance->c4_b_gpu_In, (void *)c4_c_In, 3686400UL,
               cudaMemcpyHostToDevice);
    c4_eML_blk_kernel_kernel4<<<dim3(1800U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_b_gpu_In, *chartInstance->c4_gpu_I);
    c4_I_dirtyOnGpu = true;
    c4_In_dirtyOnCpu = false;
    c4_In_dirtyOnGpu = true;
    c4_dimens1 = (int16_T)c4_position_size[0];
    c4_dimens2 = (int16_T)c4_position_size[1];
    c4_numFillColor = (int16_T)c4_b_color_size[0];
    c4_eML_blk_kernel_kernel5<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_pixCount);
    c4_pixCount_dirtyOnGpu = true;
    c4_ptrObj = NULL;
    constructDrawBaseObjectShape(&c4_ptrObj);
    c4_c_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c4_position_size[0] * c4_position_size[1] - 1) + 1L), &c4_c_grid,
      &c4_c_block, 1024U, 65535U);
    if (c4_c_validLaunchParams) {
      cudaMemcpy(chartInstance->c4_gpu_position_size, &c4_position_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      c4_position_size_dirtyOnCpu = false;
      c4_eML_blk_kernel_kernel6<<<c4_c_grid, c4_c_block>>>
        (*chartInstance->c4_gpu_position_data,
         *chartInstance->c4_gpu_position_size,
         *chartInstance->c4_gpu_positionOut_data);
      c4_positionOut_data_dirtyOnGpu = true;
    }

    c4_posPtr = NULL;
    if (c4_positionOut_data_dirtyOnGpu) {
      cudaMemcpy(&c4_positionOut_data[0], chartInstance->c4_gpu_positionOut_data,
                 448UL, cudaMemcpyDeviceToHost);
    }

    getPositionDataPointer(&c4_posPtr, &c4_positionOut_data[0], (uint32_T)
      c4_dimens1, (uint32_T)c4_dimens2);
    c4_d_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c4_b_color_size[0] * 3 - 1) + 1L), &c4_d_grid, &c4_d_block, 1024U, 65535U);
    if (c4_d_validLaunchParams) {
      if (c4_color_data_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c4_gpu_color_data, &c4_b_color_data[0], 60UL,
                   cudaMemcpyHostToDevice);
      }

      cudaMemcpy(chartInstance->c4_gpu_color_size, &c4_b_color_size[0], 8UL,
                 cudaMemcpyHostToDevice);
      c4_eML_blk_kernel_kernel7<<<c4_d_grid, c4_d_block>>>
        (*chartInstance->c4_gpu_color_data, *chartInstance->c4_gpu_color_size,
         *chartInstance->c4_b_gpu_color_data);
      c4_color_data_dirtyOnGpu = true;
    }

    c4_colPtr = NULL;
    if (c4_color_data_dirtyOnGpu) {
      cudaMemcpy(&c4_c_color_data[0], chartInstance->c4_b_gpu_color_data, 240UL,
                 cudaMemcpyDeviceToHost);
    }

    getColorDataPointer_single(&c4_colPtr, &c4_c_color_data[0], (uint32_T)
      c4_b_color_size[0], 3U);
    for (c4_i = 0; c4_i < 2; c4_i++) {
      c4_isInitialise = initialiseDrawbaseShape(c4_ptrObj, (int16_T)c4_i, 1);
      if (!c4_isInitialise) {
        if (c4_In_dirtyOnGpu) {
          cudaMemcpy((void *)c4_c_In, chartInstance->c4_b_gpu_In, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_In_dirtyOnGpu = false;
        }

        if (c4_I_dirtyOnGpu) {
          cudaMemcpy(&chartInstance->c4_I[0], chartInstance->c4_gpu_I, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_I_dirtyOnGpu = false;
        }

        if (c4_pixCount_dirtyOnGpu) {
          cudaMemcpy(&c4_pixCount[0], chartInstance->c4_gpu_pixCount, 640UL,
                     cudaMemcpyDeviceToHost);
          c4_pixCount_dirtyOnGpu = false;
        }

        instantiateDrawBaseShape_single(c4_ptrObj, &c4_c_In[0],
          &chartInstance->c4_I[0], c4_posPtr, c4_colPtr, 0.6, 1, 1, true, 480,
          640, 3, 2, c4_dimens1, c4_dimens2, c4_numFillColor, false, c4_bv[c4_i],
          &c4_pixCount[0], (int16_T)c4_i);
        c4_In_dirtyOnCpu = true;
      }
    }

    mDrawShapes(c4_ptrObj, false, true, 1, 1, 480, 640);
    deallocateMemoryShape(c4_ptrObj);
    deletePositionDataPointer(c4_posPtr);
    deleteColorDataPointer_single(c4_colPtr);
    c4_i25 = c4_position_size[0] - 1;
    c4_textLocAndWidth_size[0] = c4_position_size[0];
    c4_textLocAndWidth_size[1] = 4;
    c4_textLocAndWidth_size_dirtyOnCpu = true;
    c4_e_validLaunchParams = mwGetLaunchParameters((real_T)(((int64_T)c4_i25 +
      1L) * 4L), &c4_e_grid, &c4_e_block, 1024U, 65535U);
    if (c4_e_validLaunchParams) {
      if (c4_position_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c4_gpu_position_size, &c4_position_size[0],
                   8UL, cudaMemcpyHostToDevice);
      }

      cudaMemcpy(chartInstance->c4_gpu_textLocAndWidth_size,
                 &c4_textLocAndWidth_size[0], 8UL, cudaMemcpyHostToDevice);
      c4_textLocAndWidth_size_dirtyOnCpu = false;
      c4_eML_blk_kernel_kernel8<<<c4_e_grid, c4_e_block>>>
        (*chartInstance->c4_gpu_position_data,
         *chartInstance->c4_gpu_position_size,
         *chartInstance->c4_gpu_textLocAndWidth_size, c4_i25,
         *chartInstance->c4_b_gpu_textLocAndWidth_data);
    }

    c4_i28 = c4_position_size[0] - 1;
    c4_b_textLocAndWidth_size[0] = c4_position_size[0];
    c4_f_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c4_i28 + 1L),
      &c4_f_grid, &c4_f_block, 1024U, 65535U);
    if (c4_f_validLaunchParams) {
      if (c4_textLocAndWidth_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c4_gpu_textLocAndWidth_size,
                   &c4_textLocAndWidth_size[0], 8UL, cudaMemcpyHostToDevice);
        c4_textLocAndWidth_size_dirtyOnCpu = false;
      }

      c4_eML_blk_kernel_kernel9<<<c4_f_grid, c4_f_block>>>
        (*chartInstance->c4_b_gpu_textLocAndWidth_data,
         *chartInstance->c4_gpu_textLocAndWidth_size, c4_i28,
         *chartInstance->c4_gpu_textLocAndWidth_data);
    }

    c4_g_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
      (c4_b_textLocAndWidth_size[0] - 1) + 1L), &c4_g_grid, &c4_g_block, 1024U,
      65535U);
    if (c4_g_validLaunchParams) {
      if (c4_textLocAndWidth_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c4_gpu_textLocAndWidth_size,
                   &c4_textLocAndWidth_size[0], 8UL, cudaMemcpyHostToDevice);
        c4_textLocAndWidth_size_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c4_b_gpu_textLocAndWidth_size,
                 &c4_b_textLocAndWidth_size[0], 4UL, cudaMemcpyHostToDevice);
      c4_eML_blk_kernel_kernel10<<<c4_g_grid, c4_g_block>>>
        (*chartInstance->c4_gpu_textLocAndWidth_data,
         *chartInstance->c4_gpu_textLocAndWidth_size,
         *chartInstance->c4_b_gpu_textLocAndWidth_size,
         *chartInstance->c4_b_gpu_textLocAndWidth_data);
    }

    c4_i34 = c4_textLocAndWidth_size[0] - 1;
    c4_textPosition_size[0] = c4_textLocAndWidth_size[0];
    c4_textPosition_size[1] = 2;
    c4_h_validLaunchParams = mwGetLaunchParameters((real_T)(((int64_T)c4_i34 +
      1L) * 2L), &c4_h_grid, &c4_h_block, 1024U, 65535U);
    if (c4_h_validLaunchParams) {
      if (c4_textLocAndWidth_size_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c4_gpu_textLocAndWidth_size,
                   &c4_textLocAndWidth_size[0], 8UL, cudaMemcpyHostToDevice);
        c4_textLocAndWidth_size_dirtyOnCpu = false;
      }

      cudaMemcpy(chartInstance->c4_gpu_textPosition_size, &c4_textPosition_size
                 [0], 8UL, cudaMemcpyHostToDevice);
      c4_eML_blk_kernel_kernel11<<<c4_h_grid, c4_h_block>>>
        (*chartInstance->c4_b_gpu_textLocAndWidth_data,
         *chartInstance->c4_gpu_textLocAndWidth_size,
         *chartInstance->c4_gpu_textPosition_size, c4_i34,
         *chartInstance->c4_gpu_textPosition_data);
      c4_textPosition_data_dirtyOnGpu = true;
    }

    if (c4_color_size[0] == 1) {
      c4_textColor_size[0] = (int8_T)c4_textLocAndWidth_size[0];
      c4_c_ntilerows = (int8_T)c4_textLocAndWidth_size[0];
      for (c4_c_jcol = 0; c4_c_jcol < 3; c4_c_jcol++) {
        c4_c_ibmat = c4_c_jcol * (int8_T)c4_textLocAndWidth_size[0];
        for (c4_c_itilerow = 0; c4_c_itilerow < c4_c_ntilerows; c4_c_itilerow++)
        {
          c4_textColor_data[c4_c_ibmat + c4_c_itilerow] =
            c4_color_data[c4_c_jcol];
        }
      }
    } else {
      c4_textColor_size[0] = c4_color_size[0];
      c4_i_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
        (c4_color_size[0] * 3 - 1) + 1L), &c4_i_grid, &c4_i_block, 1024U, 65535U);
      if (c4_i_validLaunchParams) {
        if (c4_b_color_data_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c4_c_gpu_color_data, &c4_color_data[0], 60UL,
                     cudaMemcpyHostToDevice);
        }

        if (c4_color_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c4_b_gpu_color_size, &c4_color_size[0], 8UL,
                     cudaMemcpyHostToDevice);
        }

        c4_eML_blk_kernel_kernel12<<<c4_i_grid, c4_i_block>>>
          (*chartInstance->c4_c_gpu_color_data,
           *chartInstance->c4_b_gpu_color_size,
           *chartInstance->c4_gpu_textColor_data);
        c4_textColor_data_dirtyOnGpu = true;
      }
    }

    if (c4_textLocAndWidth_size[0] == 1) {
      c4_eML_blk_kernel_kernel14<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c4_b_gpu_textLocAndWidth_data,
         *chartInstance->c4_gpu_shapeWidth_data);
      c4_shapeWidth_data_dirtyOnGpu = true;
    } else {
      c4_iv[0] = c4_textLocAndWidth_size[0];
      c4_j_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c4_iv[0]
        - 1) + 1L), &c4_j_grid, &c4_j_block, 1024U, 65535U);
      if (c4_j_validLaunchParams) {
        if (c4_textLocAndWidth_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c4_gpu_textLocAndWidth_size,
                     &c4_textLocAndWidth_size[0], 8UL, cudaMemcpyHostToDevice);
          c4_textLocAndWidth_size_dirtyOnCpu = false;
        }

        cudaMemcpy(chartInstance->c4_gpu_iv, &c4_iv[0], 8UL,
                   cudaMemcpyHostToDevice);
        c4_eML_blk_kernel_kernel13<<<c4_j_grid, c4_j_block>>>
          (*chartInstance->c4_b_gpu_textLocAndWidth_data,
           *chartInstance->c4_gpu_textLocAndWidth_size,
           *chartInstance->c4_gpu_iv, *chartInstance->c4_gpu_shapeWidth_data);
        c4_shapeWidth_data_dirtyOnGpu = true;
      }
    }

    if (c4_textLocAndWidth_size[0] == 1) {
      c4_eML_blk_kernel_kernel16<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c4_b_gpu_textLocAndWidth_data,
         *chartInstance->c4_gpu_shapeHeight_data);
      c4_shapeHeight_data_dirtyOnGpu = true;
    } else {
      c4_iv1[0] = c4_textLocAndWidth_size[0];
      c4_k_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)(c4_iv1[0]
        - 1) + 1L), &c4_k_grid, &c4_k_block, 1024U, 65535U);
      if (c4_k_validLaunchParams) {
        if (c4_textLocAndWidth_size_dirtyOnCpu) {
          cudaMemcpy(chartInstance->c4_gpu_textLocAndWidth_size,
                     &c4_textLocAndWidth_size[0], 8UL, cudaMemcpyHostToDevice);
        }

        cudaMemcpy(chartInstance->c4_b_gpu_iv1, &c4_iv1[0], 8UL,
                   cudaMemcpyHostToDevice);
        c4_eML_blk_kernel_kernel15<<<c4_k_grid, c4_k_block>>>
          (*chartInstance->c4_b_gpu_textLocAndWidth_data,
           *chartInstance->c4_gpu_textLocAndWidth_size,
           *chartInstance->c4_b_gpu_iv1, *chartInstance->c4_gpu_shapeHeight_data);
        c4_shapeHeight_data_dirtyOnGpu = true;
      }
    }

    c4_isScalarText = (c4_scores_size[0] == 1);
    c4_textIdx = 0;
    c4_c_i = c4_textLocAndWidth_size[0] - 1;
    for (c4_ii = 0; c4_ii <= c4_c_i; c4_ii++) {
      if (!c4_isScalarText) {
        c4_textIdx = c4_ii;
      }

      c4_eML_blk_kernel_kernel17<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c4_gpu_str1);
      if (c4_cv8_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c4_gpu_cv8, &c4_cv8[0], 6UL,
                   cudaMemcpyHostToDevice);
        c4_cv8_dirtyOnCpu = false;
      }

      c4_eML_blk_kernel_kernel18<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c4_gpu_cv8, *chartInstance->c4_gpu_cv7);
      cudaMemcpy(&c4_str1[0], chartInstance->c4_gpu_str1, 30UL,
                 cudaMemcpyDeviceToHost);
      cudaMemcpy(&c4_cv7[0], chartInstance->c4_gpu_cv7, 6UL,
                 cudaMemcpyDeviceToHost);
      sprintf(&c4_str1[0], &c4_cv7[0], (real_T)c4_b_scores_data[c4_textIdx]);
      c4_endIdx = 0;
      c4_d_i = 0;
      c4_exitg1 = false;
      while ((!c4_exitg1) && (c4_d_i < 30)) {
        if ((uint8_T)c4_str1[c4_d_i] == 0) {
          c4_endIdx = c4_d_i;
          c4_exitg1 = true;
        } else {
          c4_d_i++;
        }
      }

      if (1 > c4_endIdx) {
        c4_i1 = -1;
      } else {
        c4_i1 = c4_endIdx - 1;
      }

      c4_thisTextU16_size[0] = 1;
      c4_thisTextU16_size[1] = c4_i1 + 1;
      c4_thisTextU16_size_dirtyOnCpu = true;
      c4_l_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c4_i1 +
        1L), &c4_l_grid, &c4_l_block, 1024U, 65535U);
      if (c4_l_validLaunchParams) {
        cudaMemcpy(chartInstance->c4_gpu_str1, &c4_str1[0], 30UL,
                   cudaMemcpyHostToDevice);
        c4_eML_blk_kernel_kernel19<<<c4_l_grid, c4_l_block>>>
          (*chartInstance->c4_gpu_str1, c4_i1,
           *chartInstance->c4_gpu_thisTextU16_data);
        c4_thisTextU16_data_dirtyOnGpu = true;
      }

      if (c4_thisTextU16_size[1] != 0) {
        c4_isNewLineChar_size[1] = c4_thisTextU16_size[1];
        c4_m_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
          (c4_thisTextU16_size[1] - 1) + 1L), &c4_m_grid, &c4_m_block, 1024U,
          65535U);
        if (c4_m_validLaunchParams) {
          cudaMemcpy(chartInstance->c4_gpu_thisTextU16_size,
                     &c4_thisTextU16_size[0], 8UL, cudaMemcpyHostToDevice);
          c4_thisTextU16_size_dirtyOnCpu = false;
          c4_eML_blk_kernel_kernel20<<<c4_m_grid, c4_m_block>>>
            (*chartInstance->c4_gpu_thisTextU16_data,
             *chartInstance->c4_gpu_thisTextU16_size,
             *chartInstance->c4_gpu_isNewLineChar_data);
          c4_isNewLineChar_data_dirtyOnGpu = true;
        }

        c4_nx = c4_isNewLineChar_size[1];
        c4_idx = 0;
        c4_ii_size[0] = 1;
        c4_ii_size[1] = c4_isNewLineChar_size[1];
        c4_b_ii = 1;
        c4_exitg1 = false;
        while ((!c4_exitg1) && (c4_b_ii <= c4_nx)) {
          if (c4_isNewLineChar_data_dirtyOnGpu) {
            cudaMemcpy(&c4_isNewLineChar_data[0],
                       chartInstance->c4_gpu_isNewLineChar_data, 29UL,
                       cudaMemcpyDeviceToHost);
            c4_isNewLineChar_data_dirtyOnGpu = false;
          }

          if (c4_isNewLineChar_data[c4_b_ii - 1]) {
            c4_idx++;
            c4_ii_data[c4_idx - 1] = (int8_T)c4_b_ii;
            c4_ii_data_dirtyOnCpu = true;
            if (c4_idx >= c4_nx) {
              c4_exitg1 = true;
            } else {
              c4_b_ii++;
            }
          } else {
            c4_b_ii++;
          }
        }

        if (c4_isNewLineChar_size[1] == 1) {
          if (c4_idx == 0) {
            c4_ii_size[0] = 1;
            c4_ii_size[1] = 0;
          }
        } else if (1 > c4_idx) {
          c4_ii_size[1] = 0;
        } else {
          c4_ii_size[1] = c4_idx;
        }

        c4_idxNewlineChar_size[1] = c4_ii_size[1];
        c4_n_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
          (c4_ii_size[1] - 1) + 1L), &c4_n_grid, &c4_n_block, 1024U, 65535U);
        if (c4_n_validLaunchParams) {
          if (c4_ii_data_dirtyOnCpu) {
            cudaMemcpy(chartInstance->c4_gpu_ii_data, &c4_ii_data[0], 29UL,
                       cudaMemcpyHostToDevice);
            c4_ii_data_dirtyOnCpu = false;
          }

          cudaMemcpy(chartInstance->c4_gpu_ii_size, &c4_ii_size[0], 8UL,
                     cudaMemcpyHostToDevice);
          c4_eML_blk_kernel_kernel21<<<c4_n_grid, c4_n_block>>>
            (*chartInstance->c4_gpu_ii_data, *chartInstance->c4_gpu_ii_size,
             *chartInstance->c4_gpu_idxNewlineChar_data);
          c4_idxNewlineChar_data_dirtyOnGpu = true;
        }

        if (c4_idxNewlineChar_size[1] == 0) {
          c4_thisCharcodes_1b_size[0] = 1;
          c4_thisCharcodes_1b_size[1] = c4_thisTextU16_size[1];
          c4_thisCharcodes_1b_size_dirtyOnCpu = true;
          c4_o_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_thisTextU16_size[1] - 1) + 1L), &c4_o_grid, &c4_o_block, 1024U,
            65535U);
          if (c4_o_validLaunchParams) {
            if (c4_thisTextU16_size_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_thisTextU16_size,
                         &c4_thisTextU16_size[0], 8UL, cudaMemcpyHostToDevice);
              c4_thisTextU16_size_dirtyOnCpu = false;
            }

            c4_eML_blk_kernel_kernel43<<<c4_o_grid, c4_o_block>>>
              (*chartInstance->c4_gpu_thisTextU16_data,
               *chartInstance->c4_gpu_thisTextU16_size,
               *chartInstance->c4_gpu_thisCharcodes_1b_data);
          }

          c4_x_size[0] = 1;
          c4_x_size[1] = c4_thisCharcodes_1b_size[1];
          c4_q_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_thisCharcodes_1b_size[1] - 1) + 1L), &c4_q_grid, &c4_q_block,
            1024U, 65535U);
          if (c4_q_validLaunchParams) {
            if (c4_iv1_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                         cudaMemcpyHostToDevice);
              c4_iv1_dirtyOnCpu = false;
            }

            if (c4_uv_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                         cudaMemcpyHostToDevice);
              c4_uv_dirtyOnCpu = false;
            }

            cudaMemcpy(chartInstance->c4_gpu_thisCharcodes_1b_size,
                       &c4_thisCharcodes_1b_size[0], 8UL, cudaMemcpyHostToDevice);
            c4_thisCharcodes_1b_size_dirtyOnCpu = false;
            c4_eML_blk_kernel_kernel44<<<c4_q_grid, c4_q_block>>>
              (*chartInstance->c4_gpu_iv1, *chartInstance->c4_gpu_uv,
               *chartInstance->c4_gpu_thisCharcodes_1b_data,
               *chartInstance->c4_gpu_thisCharcodes_1b_size,
               *chartInstance->c4_gpu_x_data);
          }

          if (c4_iv1_dirtyOnCpu) {
            cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                       cudaMemcpyHostToDevice);
            c4_iv1_dirtyOnCpu = false;
          }

          if (c4_uv_dirtyOnCpu) {
            cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                       cudaMemcpyHostToDevice);
            c4_uv_dirtyOnCpu = false;
          }

          c4_eML_blk_kernel_kernel45<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*chartInstance->c4_gpu_iv1, *chartInstance->c4_gpu_uv,
             *chartInstance->c4_gpu_thisCharcodes_1b_data,
             chartInstance->c4_gpu_y);
          c4_s_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_thisCharcodes_1b_size[1] - 2) + 1L), &c4_s_grid, &c4_s_block,
            1024U, 65535U);
          if (c4_s_validLaunchParams) {
            cudaMemcpy(chartInstance->c4_gpu_x_size, &c4_x_size[0], 8UL,
                       cudaMemcpyHostToDevice);
            c4_eML_blk_kernel_kernel46<<<c4_s_grid, c4_s_block>>>
              (*chartInstance->c4_gpu_x_size, *chartInstance->c4_gpu_x_data,
               c4_thisCharcodes_1b_size[1], chartInstance->c4_gpu_y);
          }

          c4_isNewLineChar_size[0] = 1;
          c4_isNewLineChar_size[1] = c4_thisCharcodes_1b_size[1];
          c4_t_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_thisCharcodes_1b_size[1] - 1) + 1L), &c4_t_grid, &c4_t_block,
            1024U, 65535U);
          if (c4_t_validLaunchParams) {
            if (c4_thisCharcodes_1b_size_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_thisCharcodes_1b_size,
                         &c4_thisCharcodes_1b_size[0], 8UL,
                         cudaMemcpyHostToDevice);
            }

            c4_eML_blk_kernel_kernel47<<<c4_t_grid, c4_t_block>>>
              (*chartInstance->c4_gpu_uv,
               *chartInstance->c4_gpu_thisCharcodes_1b_data,
               *chartInstance->c4_gpu_thisCharcodes_1b_size,
               *chartInstance->c4_gpu_isNewLineChar_data);
            c4_isNewLineChar_data_dirtyOnGpu = true;
          }

          c4_b_vlen = c4_isNewLineChar_size[1];
          c4_eML_blk_kernel_kernel48<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
            (*chartInstance->c4_gpu_isNewLineChar_data, chartInstance->c4_gpu_nz);
          c4_w_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_isNewLineChar_size[1] - 2) + 1L), &c4_w_grid, &c4_w_block, 1024U,
            65535U);
          if (c4_w_validLaunchParams) {
            cudaMemcpy(chartInstance->c4_gpu_isNewLineChar_size,
                       &c4_isNewLineChar_size[0], 8UL, cudaMemcpyHostToDevice);
            c4_eML_blk_kernel_kernel49<<<c4_w_grid, c4_w_block>>>
              (*chartInstance->c4_gpu_isNewLineChar_size,
               *chartInstance->c4_gpu_isNewLineChar_data, c4_b_vlen,
               chartInstance->c4_gpu_nz);
          }

          cudaMemcpy(&c4_y, chartInstance->c4_gpu_y, 8UL, cudaMemcpyDeviceToHost);
          if (c4_y < 2.147483648E+9) {
            c4_i73 = (int32_T)c4_y;
          } else {
            c4_i73 = MAX_int32_T;
          }

          cudaMemcpy(&c4_b_nz, chartInstance->c4_gpu_nz, 4UL,
                     cudaMemcpyDeviceToHost);
          c4_d2 = (real_T)c4_b_nz * 4.0;
          if (c4_d2 < 2.147483648E+9) {
            if (c4_d2 >= -2.147483648E+9) {
              c4_i75 = (int32_T)c4_d2;
            } else {
              c4_i75 = MIN_int32_T;
            }
          } else {
            c4_i75 = MAX_int32_T;
          }

          if ((c4_i73 > 0) && (c4_i75 > MAX_int32_T - c4_i73)) {
            c4_qY = MAX_int32_T;
          } else {
            c4_qY = c4_i73 + c4_i75;
          }

          c4_tbWidth = c4_qY;
        } else {
          if (c4_idxNewlineChar_data_dirtyOnGpu) {
            cudaMemcpy(&c4_idxNewlineChar_data[0],
                       chartInstance->c4_gpu_idxNewlineChar_data, 29UL,
                       cudaMemcpyDeviceToHost);
            c4_idxNewlineChar_data_dirtyOnGpu = false;
          }

          if (1 > c4_idxNewlineChar_data[0] - 1) {
            c4_i3 = -1;
          } else {
            c4_i3 = c4_idxNewlineChar_data[0] - 2;
          }

          c4_thisCharcodes_1b_size[0] = 1;
          c4_thisCharcodes_1b_size[1] = c4_i3 + 1;
          c4_thisCharcodes_1b_size_dirtyOnCpu = true;
          c4_p_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)c4_i3
            + 1L), &c4_p_grid, &c4_p_block, 1024U, 65535U);
          if (c4_p_validLaunchParams) {
            c4_eML_blk_kernel_kernel22<<<c4_p_grid, c4_p_block>>>
              (*chartInstance->c4_gpu_thisTextU16_data, c4_i3,
               *chartInstance->c4_gpu_thisCharcodes_1b_data);
          }

          c4_x_size[0] = 1;
          c4_x_size[1] = c4_thisCharcodes_1b_size[1];
          c4_r_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_thisCharcodes_1b_size[1] - 1) + 1L), &c4_r_grid, &c4_r_block,
            1024U, 65535U);
          if (c4_r_validLaunchParams) {
            if (c4_iv1_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                         cudaMemcpyHostToDevice);
              c4_iv1_dirtyOnCpu = false;
            }

            if (c4_uv_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                         cudaMemcpyHostToDevice);
              c4_uv_dirtyOnCpu = false;
            }

            cudaMemcpy(chartInstance->c4_gpu_thisCharcodes_1b_size,
                       &c4_thisCharcodes_1b_size[0], 8UL, cudaMemcpyHostToDevice);
            c4_thisCharcodes_1b_size_dirtyOnCpu = false;
            c4_eML_blk_kernel_kernel23<<<c4_r_grid, c4_r_block>>>
              (*chartInstance->c4_gpu_iv1, *chartInstance->c4_gpu_uv,
               *chartInstance->c4_gpu_thisCharcodes_1b_data,
               *chartInstance->c4_gpu_thisCharcodes_1b_size,
               *chartInstance->c4_gpu_x_data);
          }

          c4_vlen = c4_thisCharcodes_1b_size[1];
          if (c4_thisCharcodes_1b_size[1] == 0) {
            c4_lenFirstSegment = 0.0;
            c4_lenFirstSegment_dirtyOnGpu = false;
            c4_lenFirstSegment_dirtyOnCpu = true;
          } else {
            if (c4_iv1_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                         cudaMemcpyHostToDevice);
              c4_iv1_dirtyOnCpu = false;
            }

            if (c4_uv_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                         cudaMemcpyHostToDevice);
              c4_uv_dirtyOnCpu = false;
            }

            if (c4_lenFirstSegment_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_lenFirstSegment,
                         &c4_lenFirstSegment, 8UL, cudaMemcpyHostToDevice);
              c4_lenFirstSegment_dirtyOnCpu = false;
            }

            c4_eML_blk_kernel_kernel24<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*
              chartInstance->c4_gpu_iv1, *chartInstance->c4_gpu_uv,
              *chartInstance->c4_gpu_thisCharcodes_1b_data,
              chartInstance->c4_gpu_lenFirstSegment);
            c4_lenFirstSegment_dirtyOnGpu = true;
            c4_v_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
              (c4_vlen - 2) + 1L), &c4_v_grid, &c4_v_block, 1024U, 65535U);
            if (c4_v_validLaunchParams) {
              cudaMemcpy(chartInstance->c4_gpu_x_size, &c4_x_size[0], 8UL,
                         cudaMemcpyHostToDevice);
              c4_eML_blk_kernel_kernel25<<<c4_v_grid, c4_v_block>>>
                (*chartInstance->c4_gpu_x_size, *chartInstance->c4_gpu_x_data,
                 c4_vlen, chartInstance->c4_gpu_lenFirstSegment);
            }
          }

          c4_isNewLineChar_size[0] = 1;
          c4_isNewLineChar_size[1] = c4_thisCharcodes_1b_size[1];
          c4_u_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_thisCharcodes_1b_size[1] - 1) + 1L), &c4_u_grid, &c4_u_block,
            1024U, 65535U);
          if (c4_u_validLaunchParams) {
            if (c4_uv_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                         cudaMemcpyHostToDevice);
              c4_uv_dirtyOnCpu = false;
            }

            if (c4_thisCharcodes_1b_size_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_thisCharcodes_1b_size,
                         &c4_thisCharcodes_1b_size[0], 8UL,
                         cudaMemcpyHostToDevice);
            }

            c4_eML_blk_kernel_kernel26<<<c4_u_grid, c4_u_block>>>
              (*chartInstance->c4_gpu_uv,
               *chartInstance->c4_gpu_thisCharcodes_1b_data,
               *chartInstance->c4_gpu_thisCharcodes_1b_size,
               *chartInstance->c4_gpu_isNewLineChar_data);
            c4_isNewLineChar_data_dirtyOnGpu = true;
          }

          c4_c_vlen = c4_isNewLineChar_size[1];
          if (c4_isNewLineChar_size[1] == 0) {
            c4_nz = 0;
            c4_nz_dirtyOnGpu = false;
            c4_c_nz_dirtyOnCpu = true;
          } else {
            if (c4_c_nz_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_c_gpu_nz, &c4_nz, 4UL,
                         cudaMemcpyHostToDevice);
              c4_c_nz_dirtyOnCpu = false;
            }

            c4_eML_blk_kernel_kernel27<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*
              chartInstance->c4_gpu_isNewLineChar_data,
              chartInstance->c4_c_gpu_nz);
            c4_nz_dirtyOnGpu = true;
            c4_x_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
              (c4_c_vlen - 2) + 1L), &c4_x_grid, &c4_x_block, 1024U, 65535U);
            if (c4_x_validLaunchParams) {
              cudaMemcpy(chartInstance->c4_gpu_isNewLineChar_size,
                         &c4_isNewLineChar_size[0], 8UL, cudaMemcpyHostToDevice);
              c4_eML_blk_kernel_kernel28<<<c4_x_grid, c4_x_block>>>
                (*chartInstance->c4_gpu_isNewLineChar_size,
                 *chartInstance->c4_gpu_isNewLineChar_data, c4_c_vlen,
                 chartInstance->c4_c_gpu_nz);
            }
          }

          c4_maxLen = 0;
          c4_i4 = c4_idxNewlineChar_size[1];
          for (c4_e_i = 0; c4_e_i <= c4_i4 - 3; c4_e_i++) {
            if (c4_idxNewlineChar_data[c4_e_i + 1] + 1 >
                c4_idxNewlineChar_data[c4_e_i + 2] - 1) {
              c4_i6 = 0;
              c4_i8 = -1;
            } else {
              c4_i6 = c4_idxNewlineChar_data[c4_e_i + 1];
              c4_i8 = c4_idxNewlineChar_data[c4_e_i + 2] - 2;
            }

            c4_thisCharcodes_1b_size[0] = 1;
            c4_thisCharcodes_1b_size[1] = (c4_i8 - c4_i6) + 1;
            c4_thisCharcodes_1b_size_dirtyOnCpu = true;
            c4_ab_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
              (c4_i8 - c4_i6) + 1L), &c4_ab_grid, &c4_ab_block, 1024U, 65535U);
            if (c4_ab_validLaunchParams) {
              c4_eML_blk_kernel_kernel29<<<c4_ab_grid, c4_ab_block>>>
                (*chartInstance->c4_gpu_thisTextU16_data, c4_i6, c4_i8,
                 *chartInstance->c4_gpu_thisCharcodes_1b_data);
            }

            c4_x_size[0] = 1;
            c4_x_size[1] = c4_thisCharcodes_1b_size[1];
            c4_cb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
              (c4_thisCharcodes_1b_size[1] - 1) + 1L), &c4_cb_grid, &c4_cb_block,
              1024U, 65535U);
            if (c4_cb_validLaunchParams) {
              if (c4_iv1_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                           cudaMemcpyHostToDevice);
                c4_iv1_dirtyOnCpu = false;
              }

              if (c4_uv_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                           cudaMemcpyHostToDevice);
                c4_uv_dirtyOnCpu = false;
              }

              cudaMemcpy(chartInstance->c4_gpu_thisCharcodes_1b_size,
                         &c4_thisCharcodes_1b_size[0], 8UL,
                         cudaMemcpyHostToDevice);
              c4_thisCharcodes_1b_size_dirtyOnCpu = false;
              c4_eML_blk_kernel_kernel30<<<c4_cb_grid, c4_cb_block>>>
                (*chartInstance->c4_gpu_iv1, *chartInstance->c4_gpu_uv,
                 *chartInstance->c4_gpu_thisCharcodes_1b_data,
                 *chartInstance->c4_gpu_thisCharcodes_1b_size,
                 *chartInstance->c4_gpu_x_data);
            }

            c4_e_vlen = c4_thisCharcodes_1b_size[1];
            if ((c4_thisCharcodes_1b_size[1] == 0) || (c4_thisCharcodes_1b_size
                 [1] == 0)) {
              c4_lenThisSegment = 0.0;
              c4_lenThisSegment_dirtyOnGpu = false;
              c4_lenThisSegment_dirtyOnCpu = true;
            } else {
              if (c4_iv1_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                           cudaMemcpyHostToDevice);
                c4_iv1_dirtyOnCpu = false;
              }

              if (c4_uv_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                           cudaMemcpyHostToDevice);
                c4_uv_dirtyOnCpu = false;
              }

              if (c4_lenThisSegment_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_gpu_lenThisSegment,
                           &c4_lenThisSegment, 8UL, cudaMemcpyHostToDevice);
                c4_lenThisSegment_dirtyOnCpu = false;
              }

              c4_eML_blk_kernel_kernel31<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*chartInstance->c4_gpu_iv1, *chartInstance->c4_gpu_uv,
                 *chartInstance->c4_gpu_thisCharcodes_1b_data,
                 chartInstance->c4_gpu_lenThisSegment);
              c4_lenThisSegment_dirtyOnGpu = true;
              c4_gb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
                (c4_e_vlen - 2) + 1L), &c4_gb_grid, &c4_gb_block, 1024U, 65535U);
              if (c4_gb_validLaunchParams) {
                cudaMemcpy(chartInstance->c4_gpu_x_size, &c4_x_size[0], 8UL,
                           cudaMemcpyHostToDevice);
                c4_eML_blk_kernel_kernel32<<<c4_gb_grid, c4_gb_block>>>
                  (*chartInstance->c4_gpu_x_size, *chartInstance->c4_gpu_x_data,
                   c4_e_vlen, chartInstance->c4_gpu_lenThisSegment);
              }
            }

            c4_isNewLineChar_size[0] = 1;
            c4_isNewLineChar_size[1] = c4_thisCharcodes_1b_size[1];
            c4_eb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
              (c4_thisCharcodes_1b_size[1] - 1) + 1L), &c4_eb_grid, &c4_eb_block,
              1024U, 65535U);
            if (c4_eb_validLaunchParams) {
              if (c4_uv_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                           cudaMemcpyHostToDevice);
                c4_uv_dirtyOnCpu = false;
              }

              if (c4_thisCharcodes_1b_size_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_gpu_thisCharcodes_1b_size,
                           &c4_thisCharcodes_1b_size[0], 8UL,
                           cudaMemcpyHostToDevice);
              }

              c4_eML_blk_kernel_kernel33<<<c4_eb_grid, c4_eb_block>>>
                (*chartInstance->c4_gpu_uv,
                 *chartInstance->c4_gpu_thisCharcodes_1b_data,
                 *chartInstance->c4_gpu_thisCharcodes_1b_size,
                 *chartInstance->c4_gpu_isNewLineChar_data);
              c4_isNewLineChar_data_dirtyOnGpu = true;
            }

            c4_g_vlen = c4_isNewLineChar_size[1];
            if (c4_isNewLineChar_size[1] == 0) {
              c4_d_nz = 0;
              c4_b_nz_dirtyOnCpu = true;
            } else {
              if (c4_b_nz_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_d_gpu_nz, &c4_d_nz, 4UL,
                           cudaMemcpyHostToDevice);
                c4_b_nz_dirtyOnCpu = false;
              }

              c4_eML_blk_kernel_kernel34<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*chartInstance->c4_gpu_isNewLineChar_data,
                 chartInstance->c4_d_gpu_nz);
              c4_hb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
                (c4_g_vlen - 2) + 1L), &c4_hb_grid, &c4_hb_block, 1024U, 65535U);
              if (c4_hb_validLaunchParams) {
                cudaMemcpy(chartInstance->c4_gpu_isNewLineChar_size,
                           &c4_isNewLineChar_size[0], 8UL,
                           cudaMemcpyHostToDevice);
                c4_eML_blk_kernel_kernel35<<<c4_hb_grid, c4_hb_block>>>
                  (*chartInstance->c4_gpu_isNewLineChar_size,
                   *chartInstance->c4_gpu_isNewLineChar_data, c4_g_vlen,
                   chartInstance->c4_d_gpu_nz);
              }

              cudaMemcpy(&c4_d_nz, chartInstance->c4_d_gpu_nz, 4UL,
                         cudaMemcpyDeviceToHost);
            }

            c4_d3 = (real_T)c4_d_nz * 4.0;
            if (c4_d3 < 2.147483648E+9) {
              if (c4_d3 >= -2.147483648E+9) {
                c4_i87 = (int32_T)c4_d3;
              } else {
                c4_i87 = MIN_int32_T;
              }
            } else {
              c4_i87 = MAX_int32_T;
            }

            if (c4_lenThisSegment_dirtyOnGpu) {
              cudaMemcpy(&c4_lenThisSegment,
                         chartInstance->c4_gpu_lenThisSegment, 8UL,
                         cudaMemcpyDeviceToHost);
            }

            c4_d5 = c4_lenThisSegment + (real_T)c4_i87;
            if (c4_d5 < 2.147483648E+9) {
              c4_i90 = (int32_T)c4_d5;
            } else {
              c4_i90 = MAX_int32_T;
            }

            if (c4_i90 > c4_maxLen) {
              c4_maxLen = c4_i90;
            }
          }

          if (c4_idxNewlineChar_data[c4_idxNewlineChar_size[1] - 1] + 1 >
              c4_thisTextU16_size[1]) {
            c4_i5 = 0;
            c4_i7 = -1;
          } else {
            c4_i5 = c4_idxNewlineChar_data[c4_idxNewlineChar_size[1] - 1];
            c4_i7 = c4_thisTextU16_size[1] - 1;
          }

          c4_thisCharcodes_1b_size[0] = 1;
          c4_thisCharcodes_1b_size[1] = (c4_i7 - c4_i5) + 1;
          c4_thisCharcodes_1b_size_dirtyOnCpu = true;
          c4_y_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_i7 - c4_i5) + 1L), &c4_y_grid, &c4_y_block, 1024U, 65535U);
          if (c4_y_validLaunchParams) {
            c4_eML_blk_kernel_kernel36<<<c4_y_grid, c4_y_block>>>
              (*chartInstance->c4_gpu_thisTextU16_data, c4_i5, c4_i7,
               *chartInstance->c4_gpu_thisCharcodes_1b_data);
          }

          c4_x_size[0] = 1;
          c4_x_size[1] = c4_thisCharcodes_1b_size[1];
          c4_bb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_thisCharcodes_1b_size[1] - 1) + 1L), &c4_bb_grid, &c4_bb_block,
            1024U, 65535U);
          if (c4_bb_validLaunchParams) {
            if (c4_iv1_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                         cudaMemcpyHostToDevice);
              c4_iv1_dirtyOnCpu = false;
            }

            if (c4_uv_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                         cudaMemcpyHostToDevice);
              c4_uv_dirtyOnCpu = false;
            }

            cudaMemcpy(chartInstance->c4_gpu_thisCharcodes_1b_size,
                       &c4_thisCharcodes_1b_size[0], 8UL, cudaMemcpyHostToDevice);
            c4_thisCharcodes_1b_size_dirtyOnCpu = false;
            c4_eML_blk_kernel_kernel37<<<c4_bb_grid, c4_bb_block>>>
              (*chartInstance->c4_gpu_iv1, *chartInstance->c4_gpu_uv,
               *chartInstance->c4_gpu_thisCharcodes_1b_data,
               *chartInstance->c4_gpu_thisCharcodes_1b_size,
               *chartInstance->c4_gpu_x_data);
          }

          c4_d_vlen = c4_thisCharcodes_1b_size[1];
          if ((c4_thisCharcodes_1b_size[1] == 0) || (c4_thisCharcodes_1b_size[1]
               == 0)) {
            c4_lenEndSegment = 0.0;
            c4_lenEndSegment_dirtyOnGpu = false;
            c4_lenEndSegment_dirtyOnCpu = true;
          } else {
            if (c4_iv1_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                         cudaMemcpyHostToDevice);
              c4_iv1_dirtyOnCpu = false;
            }

            if (c4_uv_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                         cudaMemcpyHostToDevice);
              c4_uv_dirtyOnCpu = false;
            }

            if (c4_lenEndSegment_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_lenEndSegment, &c4_lenEndSegment,
                         8UL, cudaMemcpyHostToDevice);
              c4_lenEndSegment_dirtyOnCpu = false;
            }

            c4_eML_blk_kernel_kernel38<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*
              chartInstance->c4_gpu_iv1, *chartInstance->c4_gpu_uv,
              *chartInstance->c4_gpu_thisCharcodes_1b_data,
              chartInstance->c4_gpu_lenEndSegment);
            c4_lenEndSegment_dirtyOnGpu = true;
            c4_fb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
              (c4_d_vlen - 2) + 1L), &c4_fb_grid, &c4_fb_block, 1024U, 65535U);
            if (c4_fb_validLaunchParams) {
              cudaMemcpy(chartInstance->c4_gpu_x_size, &c4_x_size[0], 8UL,
                         cudaMemcpyHostToDevice);
              c4_eML_blk_kernel_kernel39<<<c4_fb_grid, c4_fb_block>>>
                (*chartInstance->c4_gpu_x_size, *chartInstance->c4_gpu_x_data,
                 c4_d_vlen, chartInstance->c4_gpu_lenEndSegment);
            }
          }

          c4_isNewLineChar_size[0] = 1;
          c4_isNewLineChar_size[1] = c4_thisCharcodes_1b_size[1];
          c4_db_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
            (c4_thisCharcodes_1b_size[1] - 1) + 1L), &c4_db_grid, &c4_db_block,
            1024U, 65535U);
          if (c4_db_validLaunchParams) {
            if (c4_uv_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                         cudaMemcpyHostToDevice);
              c4_uv_dirtyOnCpu = false;
            }

            if (c4_thisCharcodes_1b_size_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_thisCharcodes_1b_size,
                         &c4_thisCharcodes_1b_size[0], 8UL,
                         cudaMemcpyHostToDevice);
            }

            c4_eML_blk_kernel_kernel40<<<c4_db_grid, c4_db_block>>>
              (*chartInstance->c4_gpu_uv,
               *chartInstance->c4_gpu_thisCharcodes_1b_data,
               *chartInstance->c4_gpu_thisCharcodes_1b_size,
               *chartInstance->c4_gpu_isNewLineChar_data);
            c4_isNewLineChar_data_dirtyOnGpu = true;
          }

          c4_f_vlen = c4_isNewLineChar_size[1];
          if (c4_isNewLineChar_size[1] == 0) {
            c4_c_nz = 0;
            c4_b_nz_dirtyOnGpu = false;
            c4_nz_dirtyOnCpu = true;
          } else {
            if (c4_nz_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_b_gpu_nz, &c4_c_nz, 4UL,
                         cudaMemcpyHostToDevice);
              c4_nz_dirtyOnCpu = false;
            }

            c4_eML_blk_kernel_kernel41<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*
              chartInstance->c4_gpu_isNewLineChar_data,
              chartInstance->c4_b_gpu_nz);
            c4_b_nz_dirtyOnGpu = true;
            c4_ib_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
              (c4_f_vlen - 2) + 1L), &c4_ib_grid, &c4_ib_block, 1024U, 65535U);
            if (c4_ib_validLaunchParams) {
              cudaMemcpy(chartInstance->c4_gpu_isNewLineChar_size,
                         &c4_isNewLineChar_size[0], 8UL, cudaMemcpyHostToDevice);
              c4_eML_blk_kernel_kernel42<<<c4_ib_grid, c4_ib_block>>>
                (*chartInstance->c4_gpu_isNewLineChar_size,
                 *chartInstance->c4_gpu_isNewLineChar_data, c4_f_vlen,
                 chartInstance->c4_b_gpu_nz);
            }
          }

          if (c4_nz_dirtyOnGpu) {
            cudaMemcpy(&c4_nz, chartInstance->c4_c_gpu_nz, 4UL,
                       cudaMemcpyDeviceToHost);
          }

          c4_d4 = (real_T)c4_nz * 4.0;
          if (c4_d4 < 2.147483648E+9) {
            if (c4_d4 >= -2.147483648E+9) {
              c4_i88 = (int32_T)c4_d4;
            } else {
              c4_i88 = MIN_int32_T;
            }
          } else {
            c4_i88 = MAX_int32_T;
          }

          if (c4_lenFirstSegment_dirtyOnGpu) {
            cudaMemcpy(&c4_lenFirstSegment,
                       chartInstance->c4_gpu_lenFirstSegment, 8UL,
                       cudaMemcpyDeviceToHost);
          }

          c4_d6 = c4_lenFirstSegment + (real_T)c4_i88;
          if (c4_d6 < 2.147483648E+9) {
            c4_i91 = (int32_T)c4_d6;
          } else {
            c4_i91 = MAX_int32_T;
          }

          c4_varargin_1[0] = c4_i91;
          c4_varargin_1[1] = c4_maxLen;
          if (c4_b_nz_dirtyOnGpu) {
            cudaMemcpy(&c4_c_nz, chartInstance->c4_b_gpu_nz, 4UL,
                       cudaMemcpyDeviceToHost);
          }

          c4_d7 = (real_T)c4_c_nz * 4.0;
          if (c4_d7 < 2.147483648E+9) {
            if (c4_d7 >= -2.147483648E+9) {
              c4_i93 = (int32_T)c4_d7;
            } else {
              c4_i93 = MIN_int32_T;
            }
          } else {
            c4_i93 = MAX_int32_T;
          }

          if (c4_lenEndSegment_dirtyOnGpu) {
            cudaMemcpy(&c4_lenEndSegment, chartInstance->c4_gpu_lenEndSegment,
                       8UL, cudaMemcpyDeviceToHost);
          }

          c4_d8 = c4_lenEndSegment + (real_T)c4_i93;
          if (c4_d8 < 2.147483648E+9) {
            c4_i95 = (int32_T)c4_d8;
          } else {
            c4_i95 = MAX_int32_T;
          }

          c4_varargin_1[2] = c4_i95;
          c4_maxLen = c4_i91;
          for (c4_k_i = 0; c4_k_i < 2; c4_k_i++) {
            if (c4_maxLen < c4_varargin_1[c4_k_i + 1]) {
              c4_maxLen = c4_varargin_1[c4_k_i + 1];
            }
          }

          c4_tbWidth = c4_maxLen;
        }

        if (c4_shapeWidth_data_dirtyOnGpu) {
          cudaMemcpy(&c4_shapeWidth_data[0],
                     chartInstance->c4_gpu_shapeWidth_data, 80UL,
                     cudaMemcpyDeviceToHost);
          c4_shapeWidth_data_dirtyOnGpu = false;
        }

        c4_tbWidth = muIntScalarMax_sint32(c4_tbWidth, c4_shapeWidth_data[c4_ii]);
        c4_b_tbWidth = c4_tbWidth;
        if (c4_tbWidth > c4_shapeWidth_data[c4_ii]) {
          if (c4_tbWidth > 2147483639) {
            c4_b_qY = MAX_int32_T;
          } else {
            c4_b_qY = c4_tbWidth + 8;
          }

          c4_b_tbWidth = c4_b_qY;
        }

        c4_tbHeight = 14 * c4_idxNewlineChar_size[1] + 23;
        if (c4_textPosition_data_dirtyOnGpu) {
          cudaMemcpy(&c4_textPosition_data[0],
                     chartInstance->c4_gpu_textPosition_data, 160UL,
                     cudaMemcpyDeviceToHost);
          c4_textPosition_data_dirtyOnGpu = false;
        }

        c4_q0 = c4_textPosition_data[c4_ii + c4_textPosition_size[0]];
        if ((c4_q0 >= 0) && (c4_tbHeight < c4_q0 - MAX_int32_T)) {
          c4_c_qY = MAX_int32_T;
        } else if ((c4_q0 < 0) && (c4_tbHeight > c4_q0 - MIN_int32_T)) {
          c4_c_qY = MIN_int32_T;
        } else {
          c4_c_qY = c4_q0 - c4_tbHeight;
        }

        c4_tbTopLeftY = c4_c_qY + 1;
        c4_tbTopLeftX = c4_textPosition_data[c4_ii];
        if (c4_shapeWidth_data[c4_ii] > 0) {
          if (c4_shapeHeight_data_dirtyOnGpu) {
            cudaMemcpy(&c4_shapeHeight_data[0],
                       chartInstance->c4_gpu_shapeHeight_data, 80UL,
                       cudaMemcpyDeviceToHost);
            c4_shapeHeight_data_dirtyOnGpu = false;
          }

          if (c4_shapeHeight_data[c4_ii] > 0) {
            if ((c4_c_qY + 1 < 0) && (c4_tbHeight < MAX_int32_T - c4_c_qY)) {
              c4_e_qY = MIN_int32_T;
            } else if ((c4_c_qY + 1 > 0) && (c4_tbHeight > 2147483646 - c4_c_qY))
            {
              c4_e_qY = MAX_int32_T;
            } else {
              c4_e_qY = (c4_c_qY + c4_tbHeight) + 1;
            }

            c4_guard1 = false;
            if (c4_textPosition_data[c4_ii] <= 640) {
              c4_b_q0 = c4_textPosition_data[c4_ii];
              c4_q1 = c4_shapeWidth_data[c4_ii];
              if ((c4_b_q0 < 0) && (c4_q1 < MIN_int32_T - c4_b_q0)) {
                c4_h_qY = MIN_int32_T;
              } else if ((c4_b_q0 > 0) && (c4_q1 > MAX_int32_T - c4_b_q0)) {
                c4_h_qY = MAX_int32_T;
              } else {
                c4_h_qY = c4_b_q0 + c4_q1;
              }

              if (c4_h_qY < -2147483647) {
                c4_i_qY = MIN_int32_T;
              } else {
                c4_i_qY = c4_h_qY - 1;
              }

              if ((c4_i_qY >= 1) && (c4_e_qY <= 480)) {
                c4_b_q1 = c4_shapeHeight_data[c4_ii];
                if ((c4_e_qY < 0) && (c4_b_q1 < MIN_int32_T - c4_e_qY)) {
                  c4_k_qY = MIN_int32_T;
                } else if ((c4_e_qY > 0) && (c4_b_q1 > MAX_int32_T - c4_e_qY)) {
                  c4_k_qY = MAX_int32_T;
                } else {
                  c4_k_qY = c4_e_qY + c4_b_q1;
                }

                if (c4_k_qY < -2147483647) {
                  c4_m_qY = MIN_int32_T;
                } else {
                  c4_m_qY = c4_k_qY - 1;
                }

                if (c4_m_qY >= 1) {
                  if (c4_c_qY + 1 < 1) {
                    c4_c_q0 = (c4_c_qY + c4_tbHeight) + 1;
                    c4_c_q1 = c4_shapeHeight_data[c4_ii];
                    if ((c4_c_q0 < 0) && (c4_c_q1 < MIN_int32_T - c4_c_q0)) {
                      c4_p_qY = MIN_int32_T;
                    } else if ((c4_c_q0 > 0) && (c4_c_q1 > MAX_int32_T - c4_c_q0))
                    {
                      c4_p_qY = MAX_int32_T;
                    } else {
                      c4_p_qY = c4_c_q0 + c4_c_q1;
                    }

                    if (c4_p_qY >= 1) {
                      c4_e_q0 = c4_textPosition_data[c4_ii +
                        c4_textPosition_size[0]];
                      c4_d_q1 = c4_shapeHeight_data[c4_ii];
                      if ((c4_e_q0 < 0) && (c4_d_q1 < MIN_int32_T - c4_e_q0)) {
                        c4_q_qY = MIN_int32_T;
                      } else if ((c4_e_q0 > 0) && (c4_d_q1 > MAX_int32_T
                                  - c4_e_q0)) {
                        c4_q_qY = MAX_int32_T;
                      } else {
                        c4_q_qY = c4_e_q0 + c4_d_q1;
                      }

                      if (c4_q_qY > 2147483646) {
                        c4_t_qY = MAX_int32_T;
                      } else {
                        c4_t_qY = c4_q_qY + 1;
                      }

                      c4_tbTopLeftY = c4_t_qY;
                    }
                  }

                  c4_d_q0 = c4_textPosition_data[c4_ii];
                  if ((c4_d_q0 < 0) && (c4_b_tbWidth < MIN_int32_T - c4_d_q0)) {
                    c4_o_qY = MIN_int32_T;
                  } else if ((c4_d_q0 > 0) && (c4_b_tbWidth > MAX_int32_T
                              - c4_d_q0)) {
                    c4_o_qY = MAX_int32_T;
                  } else {
                    c4_o_qY = c4_d_q0 + c4_b_tbWidth;
                  }

                  if ((real_T)c4_o_qY - 640.0 >= -2.147483648E+9) {
                    c4_i102 = c4_o_qY - 640;
                  } else {
                    c4_i102 = MIN_int32_T;
                  }

                  if ((c4_i102 > 0) && (c4_textPosition_data[c4_ii] <= 640)) {
                    c4_f_q0 = c4_textPosition_data[c4_ii];
                    if ((c4_f_q0 >= 0) && (c4_i102 < c4_f_q0 - MAX_int32_T)) {
                      c4_s_qY = MAX_int32_T;
                    } else if ((c4_f_q0 < 0) && (c4_i102 > c4_f_q0 - MIN_int32_T))
                    {
                      c4_s_qY = MIN_int32_T;
                    } else {
                      c4_s_qY = c4_f_q0 - c4_i102;
                    }

                    c4_tbTopLeftX = c4_s_qY + 1;
                  }

                  if (c4_tbTopLeftX < 1) {
                    c4_g_q0 = c4_textPosition_data[c4_ii];
                    c4_e_q1 = c4_shapeWidth_data[c4_ii];
                    if ((c4_g_q0 < 0) && (c4_e_q1 < MIN_int32_T - c4_g_q0)) {
                      c4_u_qY = MIN_int32_T;
                    } else if ((c4_g_q0 > 0) && (c4_e_q1 > MAX_int32_T - c4_g_q0))
                    {
                      c4_u_qY = MAX_int32_T;
                    } else {
                      c4_u_qY = c4_g_q0 + c4_e_q1;
                    }

                    if (c4_u_qY >= 1) {
                      c4_tbTopLeftX = 1;
                    }
                  }
                } else {
                  c4_guard1 = true;
                }
              } else {
                c4_guard1 = true;
              }
            } else {
              c4_guard1 = true;
            }

            if (c4_guard1) {
              c4_tbTopLeftY = -32767;
              c4_tbTopLeftX = -32767;
            }
          }
        }

        c4_startR = c4_tbTopLeftY;
        if ((c4_tbTopLeftY < 0) && (c4_tbHeight < MIN_int32_T - c4_tbTopLeftY))
        {
          c4_d_qY = MIN_int32_T;
        } else if ((c4_tbTopLeftY > 0) && (c4_tbHeight > MAX_int32_T
                    - c4_tbTopLeftY)) {
          c4_d_qY = MAX_int32_T;
        } else {
          c4_d_qY = c4_tbTopLeftY + c4_tbHeight;
        }

        c4_endR = c4_d_qY - 1;
        c4_startC = c4_tbTopLeftX;
        if ((c4_tbTopLeftX < 0) && (c4_b_tbWidth < MIN_int32_T - c4_tbTopLeftX))
        {
          c4_f_qY = MIN_int32_T;
        } else if ((c4_tbTopLeftX > 0) && (c4_b_tbWidth > MAX_int32_T
                    - c4_tbTopLeftX)) {
          c4_f_qY = MAX_int32_T;
        } else {
          c4_f_qY = c4_tbTopLeftX + c4_b_tbWidth;
        }

        if (c4_f_qY < -2147483647) {
          c4_g_qY = MIN_int32_T;
        } else {
          c4_g_qY = c4_f_qY - 1;
        }

        c4_endC = c4_g_qY;
        if ((c4_tbTopLeftY > 480) || (c4_d_qY - 1 < 1) || (c4_tbTopLeftX > 640) ||
            (c4_g_qY < 1)) {
        } else {
          if (c4_tbTopLeftY < 1) {
            c4_startR = 1;
          }

          if (c4_d_qY - 1 > 480) {
            c4_endR = 480;
          }

          if (c4_tbTopLeftX < 1) {
            c4_startC = 1;
          }

          if (c4_g_qY > 640) {
            c4_endC = 640;
          }

          for (c4_i_i = 0; c4_i_i < 3; c4_i_i++) {
            for (c4_c = 0; c4_c <= c4_endC - c4_startC; c4_c++) {
              c4_b_c = (c4_startC + c4_c) - 1;
              for (c4_r = 0; c4_r <= c4_endR - c4_startR; c4_r++) {
                c4_b_r = (c4_startR + c4_r) - 1;
                if (c4_textColor_data_dirtyOnGpu) {
                  cudaMemcpy(&c4_textColor_data[0],
                             chartInstance->c4_gpu_textColor_data, 60UL,
                             cudaMemcpyDeviceToHost);
                  c4_textColor_data_dirtyOnGpu = false;
                }

                c4_tmp11 = (real32_T)(0.6 * (real_T)c4_textColor_data[c4_ii +
                                      c4_textColor_size[0] * c4_i_i]);
                if (c4_In_dirtyOnGpu) {
                  cudaMemcpy((void *)c4_c_In, chartInstance->c4_b_gpu_In,
                             3686400UL, cudaMemcpyDeviceToHost);
                  c4_In_dirtyOnGpu = false;
                }

                c4_tmp22 = (real32_T)(0.4 * (real_T)c4_c_In[(c4_b_r + 480 *
                  c4_b_c) + 307200 * c4_i_i]);
                c4_c_In[(c4_b_r + 480 * c4_b_c) + 307200 * c4_i_i] = c4_tmp11 +
                  c4_tmp22;
                c4_In_dirtyOnCpu = true;
              }
            }
          }
        }

        if (c4_tbTopLeftX > 2147483643) {
          c4_j_qY = MAX_int32_T;
        } else {
          c4_j_qY = c4_tbTopLeftX + 4;
        }

        c4_penX = c4_j_qY;
        if (c4_tbTopLeftY > 2147483643) {
          c4_l_qY = MAX_int32_T;
        } else {
          c4_l_qY = c4_tbTopLeftY + 4;
        }

        if (c4_l_qY > 2147483635) {
          c4_n_qY = MAX_int32_T;
        } else {
          c4_n_qY = c4_l_qY + 12;
        }

        c4_penY = c4_n_qY;
        c4_jb_validLaunchParams = mwGetLaunchParameters((real_T)((int64_T)
          (c4_thisTextU16_size[1] - 1) + 1L), &c4_jb_grid, &c4_jb_block, 1024U,
          65535U);
        if (c4_jb_validLaunchParams) {
          if (c4_thisTextU16_size_dirtyOnCpu) {
            cudaMemcpy(chartInstance->c4_gpu_thisTextU16_size,
                       &c4_thisTextU16_size[0], 8UL, cudaMemcpyHostToDevice);
          }

          c4_eML_blk_kernel_kernel50<<<c4_jb_grid, c4_jb_block>>>
            (*chartInstance->c4_gpu_thisTextU16_data,
             *chartInstance->c4_gpu_thisTextU16_size,
             *chartInstance->c4_gpu_isNewLineChar_data);
          c4_isNewLineChar_data_dirtyOnGpu = true;
        }

        c4_i9 = c4_thisTextU16_size[1];
        for (c4_j_i = 0; c4_j_i < c4_i9; c4_j_i++) {
          if (c4_isNewLineChar_data_dirtyOnGpu) {
            cudaMemcpy(&c4_isNewLineChar_data[0],
                       chartInstance->c4_gpu_isNewLineChar_data, 29UL,
                       cudaMemcpyDeviceToHost);
            c4_isNewLineChar_data_dirtyOnGpu = false;
          }

          if (c4_isNewLineChar_data[c4_j_i]) {
            if (c4_penY > 2147483633) {
              c4_r_qY = MAX_int32_T;
            } else {
              c4_r_qY = c4_penY + 14;
            }

            c4_penY = c4_r_qY;
            c4_penX = c4_j_qY;
          } else {
            if (c4_uv_dirtyOnCpu) {
              cudaMemcpy(chartInstance->c4_gpu_uv, &c4_uv[0], 512UL,
                         cudaMemcpyHostToDevice);
              c4_uv_dirtyOnCpu = false;
            }

            c4_eML_blk_kernel_kernel51<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>(*
              chartInstance->c4_gpu_uv, c4_j_i,
              *chartInstance->c4_gpu_thisTextU16_data,
              chartInstance->c4_gpu_thisGlyphIdx_1b);
            if (c4_thisTextU16_data_dirtyOnGpu) {
              cudaMemcpy(&c4_thisTextU16_data[0],
                         chartInstance->c4_gpu_thisTextU16_data, 29UL,
                         cudaMemcpyDeviceToHost);
              c4_thisTextU16_data_dirtyOnGpu = false;
            }

            if (c4_uv[c4_thisTextU16_data[c4_j_i]] == 0) {
              if (c4_penX > 2147483643) {
                c4_v_qY = MAX_int32_T;
              } else {
                c4_v_qY = c4_penX + 4;
              }

              c4_penX = c4_v_qY;
            } else {
              c4_f_q1 = c4_iv2[c4_uv[c4_thisTextU16_data[c4_j_i]]];
              if ((c4_penX < 0) && (c4_f_q1 < MIN_int32_T - c4_penX)) {
                c4_w_qY = MIN_int32_T;
              } else if ((c4_penX > 0) && (c4_f_q1 > MAX_int32_T - c4_penX)) {
                c4_w_qY = MAX_int32_T;
              } else {
                c4_w_qY = c4_penX + c4_f_q1;
              }

              c4_yy = c4_penY - c4_iv3[c4_uv[c4_thisTextU16_data[c4_j_i]]];
              c4_startR_im = c4_yy;
              c4_g_q1 = c4_uv1[c4_uv[c4_thisTextU16_data[c4_j_i]]];
              if (c4_yy > MAX_int32_T - c4_g_q1) {
                c4_x_qY = MAX_int32_T;
              } else {
                c4_x_qY = c4_yy + c4_g_q1;
              }

              c4_endR_im = c4_x_qY - 1;
              c4_startC_im = c4_w_qY;
              c4_h_q1 = c4_uv2[c4_uv[c4_thisTextU16_data[c4_j_i]]];
              if (c4_w_qY > MAX_int32_T - c4_h_q1) {
                c4_y_qY = MAX_int32_T;
              } else {
                c4_y_qY = c4_w_qY + c4_h_q1;
              }

              c4_endC_im = c4_y_qY - 1;
              if ((c4_yy > 480) || (c4_x_qY - 1 < 1) || (c4_w_qY > 640) ||
                  (c4_y_qY - 1 < 1)) {
              } else {
                c4_startR_gl = 1;
                c4_startC_gl = 1;
                c4_endR_gl = c4_uv1[c4_uv[c4_thisTextU16_data[c4_j_i]]];
                c4_endC_gl = c4_uv2[c4_uv[c4_thisTextU16_data[c4_j_i]]];
                if (c4_yy < 1) {
                  c4_startR_gl = 2 - c4_yy;
                  c4_startR_im = 1;
                }

                if (c4_x_qY - 1 > 480) {
                  c4_endR_gl = (c4_uv1[c4_uv[c4_thisTextU16_data[c4_j_i]]] -
                                c4_x_qY) + 481;
                  c4_endR_im = 480;
                }

                if (c4_w_qY < 1) {
                  if (-c4_w_qY > 2147483645) {
                    c4_bb_qY = MAX_int32_T;
                  } else {
                    c4_bb_qY = 2 - c4_w_qY;
                  }

                  c4_startC_gl = c4_bb_qY;
                  c4_startC_im = 1;
                }

                if (c4_y_qY - 1 > 640) {
                  c4_endC_gl = (c4_uv2[c4_uv[c4_thisTextU16_data[c4_j_i]]] -
                                c4_y_qY) + 641;
                  c4_endC_im = 640;
                }

                c4_bitmapEndIdx_1b = (int32_T)((uint32_T)
                  c4_uv3[c4_uv[c4_thisTextU16_data[c4_j_i]]] + (uint32_T)
                  (c4_uv2[c4_uv[c4_thisTextU16_data[c4_j_i]]] *
                   c4_uv1[c4_uv[c4_thisTextU16_data[c4_j_i]]]));
                if ((uint32_T)c4_uv3[c4_uv[c4_thisTextU16_data[c4_j_i]]] + 1U >
                    (uint32_T)c4_bitmapEndIdx_1b) {
                  c4_i10 = 0;
                  c4_i11 = -1;
                } else {
                  c4_i10 = c4_uv3[c4_uv[c4_thisTextU16_data[c4_j_i]]];
                  c4_i11 = c4_bitmapEndIdx_1b - 1;
                }

                c4_outsize_idx_0 = c4_uv2[c4_uv[c4_thisTextU16_data[c4_j_i]]];
                c4_outsize_idx_1 = c4_uv1[c4_uv[c4_thisTextU16_data[c4_j_i]]];
                if (c4_startR_gl > c4_endR_gl) {
                  c4_i12 = 0;
                  c4_i13 = -1;
                } else {
                  c4_i12 = c4_startR_gl - 1;
                  c4_i13 = c4_endR_gl - 1;
                }

                if (c4_startC_gl > c4_endC_gl) {
                  c4_i14 = 0;
                  c4_i15 = -1;
                } else {
                  c4_i14 = c4_startC_gl - 1;
                  c4_i15 = c4_endC_gl - 1;
                }

                c4_kb_validLaunchParams = mwGetLaunchParameters((real_T)
                  ((int64_T)(c4_i11 - c4_i10) + 1L), &c4_kb_grid, &c4_kb_block,
                  1024U, 65535U);
                if (c4_kb_validLaunchParams) {
                  if (c4_uv4_dirtyOnCpu) {
                    cudaMemcpy(chartInstance->c4_gpu_uv4, &c4_uv4[0], 10664UL,
                               cudaMemcpyHostToDevice);
                    c4_uv4_dirtyOnCpu = false;
                  }

                  c4_eML_blk_kernel_kernel52<<<c4_kb_grid, c4_kb_block>>>
                    (*chartInstance->c4_gpu_uv4, c4_i10, c4_i11,
                     *chartInstance->c4_gpu_uv4_data);
                }

                c4_outsize[0] = c4_outsize_idx_0;
                c4_outsize[1] = c4_outsize_idx_1;
                c4_uv4_size[0] = c4_outsize_idx_1;
                c4_uv4_size[1] = c4_outsize_idx_0;
                c4_lb_validLaunchParams = mwGetLaunchParameters((real_T)
                  (((int64_T)(c4_outsize[1] - 1) + 1L) * ((int64_T)(c4_outsize[0]
                  - 1) + 1L)), &c4_lb_grid, &c4_lb_block, 1024U, 65535U);
                if (c4_lb_validLaunchParams) {
                  cudaMemcpy(chartInstance->c4_gpu_uv4_size, &c4_uv4_size[0],
                             8UL, cudaMemcpyHostToDevice);
                  cudaMemcpy(chartInstance->c4_b_gpu_outsize, &c4_outsize[0],
                             2UL, cudaMemcpyHostToDevice);
                  c4_eML_blk_kernel_kernel53<<<c4_lb_grid, c4_lb_block>>>
                    (*chartInstance->c4_gpu_uv4_data,
                     *chartInstance->c4_gpu_uv4_size,
                     *chartInstance->c4_b_gpu_outsize,
                     *chartInstance->c4_b_gpu_uv4_data);
                }

                c4_eML_blk_kernel_kernel54<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                  (c4_outsize_idx_1, *chartInstance->c4_gpu_outsize);
                c4_thisGlyphCut_float_size[0] = (c4_i13 - c4_i12) + 1;
                c4_thisGlyphCut_float_size[1] = (c4_i15 - c4_i14) + 1;
                c4_mb_validLaunchParams = mwGetLaunchParameters((real_T)
                  (((int64_T)(c4_i13 - c4_i12) + 1L) * ((int64_T)(c4_i15 -
                  c4_i14) + 1L)), &c4_mb_grid, &c4_mb_block, 1024U, 65535U);
                if (c4_mb_validLaunchParams) {
                  cudaMemcpy(chartInstance->c4_gpu_thisGlyphCut_float_size,
                             &c4_thisGlyphCut_float_size[0], 8UL,
                             cudaMemcpyHostToDevice);
                  c4_eML_blk_kernel_kernel55<<<c4_mb_grid, c4_mb_block>>>
                    (*chartInstance->c4_b_gpu_uv4_data,
                     *chartInstance->c4_gpu_outsize,
                     *chartInstance->c4_gpu_thisGlyphCut_float_size, c4_i12,
                     c4_i13, c4_i14, c4_i15,
                     *chartInstance->c4_gpu_thisGlyphCut_float_data);
                  c4_thisGlyphCut_float_data_dirtyOnGpu = true;
                }

                for (c4_b_idx = 0; c4_b_idx < 3; c4_b_idx++) {
                  c4_cg = 1.0;
                  for (c4_c_c = 0; c4_c_c <= c4_endC_im - c4_startC_im; c4_c_c++)
                  {
                    c4_d_c = (c4_startC_im + c4_c_c) - 1;
                    c4_rg = 1.0;
                    for (c4_c_r = 0; c4_c_r <= c4_endR_im - c4_startR_im; c4_c_r
                         ++) {
                      c4_d_r = (c4_startR_im + c4_c_r) - 1;
                      if (c4_thisGlyphCut_float_data_dirtyOnGpu) {
                        cudaMemcpy(&c4_thisGlyphCut_float_data[0],
                                   chartInstance->c4_gpu_thisGlyphCut_float_data,
                                   1056UL, cudaMemcpyDeviceToHost);
                        c4_thisGlyphCut_float_data_dirtyOnGpu = false;
                      }

                      c4_glyphVal = c4_thisGlyphCut_float_data[((int32_T)c4_rg +
                        c4_thisGlyphCut_float_size[0] * ((int32_T)c4_cg - 1)) -
                        1];
                      if (c4_glyphVal == 1.0) {
                        if (c4_In_dirtyOnGpu) {
                          cudaMemcpy((void *)c4_c_In, chartInstance->c4_b_gpu_In,
                                     3686400UL, cudaMemcpyDeviceToHost);
                          c4_In_dirtyOnGpu = false;
                        }

                        c4_c_In[(c4_d_r + 480 * c4_d_c) + 307200 * c4_b_idx] =
                          0.0F;
                        c4_In_dirtyOnCpu = true;
                      } else {
                        if (c4_glyphVal != 0.0) {
                          if (c4_In_dirtyOnGpu) {
                            cudaMemcpy((void *)c4_c_In,
                                       chartInstance->c4_b_gpu_In, 3686400UL,
                                       cudaMemcpyDeviceToHost);
                            c4_In_dirtyOnGpu = false;
                          }

                          c4_c_In[(c4_d_r + 480 * c4_d_c) + 307200 * c4_b_idx] +=
                            (0.0F - c4_c_In[(c4_d_r + 480 * c4_d_c) + 307200 *
                             c4_b_idx]) * (real32_T)c4_glyphVal;
                          c4_In_dirtyOnCpu = true;
                        }
                      }

                      c4_rg++;
                    }

                    c4_cg++;
                  }
                }
              }

              if (c4_iv1_dirtyOnCpu) {
                cudaMemcpy(chartInstance->c4_gpu_iv1, &c4_b_iv1[0], 261UL,
                           cudaMemcpyHostToDevice);
                c4_iv1_dirtyOnCpu = false;
              }

              c4_eML_blk_kernel_kernel56<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
                (*chartInstance->c4_gpu_iv1,
                 chartInstance->c4_gpu_thisGlyphIdx_1b, chartInstance->c4_gpu_q1);
              cudaMemcpy(&c4_i_q1, chartInstance->c4_gpu_q1, 4UL,
                         cudaMemcpyDeviceToHost);
              c4_q1_dirtyOnGpu = false;
              if ((c4_penX < 0) && (c4_i_q1 < MIN_int32_T - c4_penX)) {
                c4_ab_qY = MIN_int32_T;
              } else {
                if (c4_q1_dirtyOnGpu) {
                  cudaMemcpy(&c4_i_q1, chartInstance->c4_gpu_q1, 4UL,
                             cudaMemcpyDeviceToHost);
                }

                if ((c4_penX > 0) && (c4_i_q1 > MAX_int32_T - c4_penX)) {
                  c4_ab_qY = MAX_int32_T;
                } else {
                  c4_ab_qY = c4_penX + c4_i_q1;
                }
              }

              c4_penX = c4_ab_qY;
            }
          }
        }
      }
    }
  }

  if (c4_b_laneFound) {
    c4_prevpt[0] = c4_b_ltPts[0];
    c4_prevpt[1] = c4_b_ltPts[28];
    for (c4_k = 0; c4_k < 27; c4_k++) {
      cudaMemcpy(chartInstance->c4_gpu_prevpt, &c4_prevpt[0], 8UL,
                 cudaMemcpyHostToDevice);
      c4_eML_blk_kernel_kernel57<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c4_gpu_prevpt, c4_k + 1, *chartInstance->c4_gpu_pts);
      if (c4_ltPts_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c4_gpu_ltPts, &c4_b_ltPts[0], 224UL,
                   cudaMemcpyHostToDevice);
        c4_ltPts_dirtyOnCpu = false;
      }

      c4_eML_blk_kernel_kernel58<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c4_gpu_ltPts, c4_k, c4_k + 1,
         *chartInstance->c4_gpu_pts);
      c4_prevpt[0] = c4_b_ltPts[c4_k + 1];
      c4_prevpt[1] = c4_b_ltPts[c4_k + 29];
    }

    c4_eML_blk_kernel_kernel59<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_pts, *chartInstance->c4_gpu_positionOut);
    c4_pos.size[0] = 28;
    c4_pos.data[0].f1.size[0] = 4;
    cudaMemcpy(chartInstance->c4_gpu_pos, &c4_pos, 564UL, cudaMemcpyHostToDevice);
    c4_eML_blk_kernel_kernel60<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel61<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel62<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel63<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel64<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel65<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel66<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel67<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel68<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel69<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel70<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel71<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel72<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel73<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel74<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel75<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel76<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel77<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel78<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel79<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel80<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel81<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel82<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel83<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel84<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel85<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel86<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel87<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel88<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel89<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel90<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel91<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel92<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel93<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel94<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel95<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel96<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel97<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel98<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel99<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel100<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel101<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel102<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel103<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel104<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel105<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel106<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel107<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel108<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel109<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel110<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel111<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel112<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel113<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel114<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel115<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos, *chartInstance->c4_b_gpu_positionOut_data);
    c4_b_positionOut_data_dirtyOnGpu = true;
    if (c4_In_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c4_b_gpu_In, (void *)c4_c_In, 3686400UL,
                 cudaMemcpyHostToDevice);
      c4_In_dirtyOnCpu = false;
    }

    c4_eML_blk_kernel_kernel116<<<dim3(1800U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_b_gpu_In, *chartInstance->c4_gpu_I);
    c4_I_dirtyOnGpu = true;
    c4_In_dirtyOnGpu = true;
    c4_eML_blk_kernel_kernel117<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_pixCount);
    c4_pixCount_dirtyOnGpu = true;
    c4_b_ptrObj = NULL;
    constructDrawBaseObjectShape(&c4_b_ptrObj);
    c4_eML_blk_kernel_kernel118<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*chartInstance->c4_b_gpu_positionOut_data,
       *chartInstance->c4_gpu_positionOut_data);
    c4_b_posPtr = NULL;
    cudaMemcpy(&c4_positionOut_data[0], chartInstance->c4_gpu_positionOut_data,
               448UL, cudaMemcpyDeviceToHost);
    getPositionDataPointer(&c4_b_posPtr, &c4_positionOut_data[0], 28U, 4U);
    cudaMemcpy(chartInstance->c4_gpu_color, &c4_color[0], 84UL,
               cudaMemcpyHostToDevice);
    c4_color_dirtyOnCpu = false;
    c4_eML_blk_kernel_kernel119<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_color, *chartInstance->c4_b_gpu_color);
    c4_b_colPtr = NULL;
    cudaMemcpy(&c4_b_color[0], chartInstance->c4_b_gpu_color, 336UL,
               cudaMemcpyDeviceToHost);
    getColorDataPointer_single(&c4_b_colPtr, &c4_b_color[0], 28U, 3U);
    for (c4_b_i = 0; c4_b_i < 2; c4_b_i++) {
      c4_b_isInitialise = initialiseDrawbaseShape(c4_b_ptrObj, (int16_T)c4_b_i,
        2);
      if (!c4_b_isInitialise) {
        if (c4_In_dirtyOnGpu) {
          cudaMemcpy((void *)c4_c_In, chartInstance->c4_b_gpu_In, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_In_dirtyOnGpu = false;
        }

        if (c4_I_dirtyOnGpu) {
          cudaMemcpy(&chartInstance->c4_I[0], chartInstance->c4_gpu_I, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_I_dirtyOnGpu = false;
        }

        if (c4_pixCount_dirtyOnGpu) {
          cudaMemcpy(&c4_pixCount[0], chartInstance->c4_gpu_pixCount, 640UL,
                     cudaMemcpyDeviceToHost);
          c4_pixCount_dirtyOnGpu = false;
        }

        instantiateDrawBaseShape_single(c4_b_ptrObj, &c4_c_In[0],
          &chartInstance->c4_I[0], c4_b_posPtr, c4_b_colPtr, 0.6, 2, 2, true,
          480, 640, 3, 2, 28, 4, 28, false, c4_bv[c4_b_i], &c4_pixCount[0],
          (int16_T)c4_b_i);
        c4_In_dirtyOnCpu = true;
      }
    }

    mDrawShapes(c4_b_ptrObj, false, true, 2, 2, 480, 640);
    deallocateMemoryShape(c4_b_ptrObj);
    deletePositionDataPointer(c4_b_posPtr);
    deleteColorDataPointer_single(c4_b_colPtr);
    c4_prevpt[0] = c4_b_rtPts[0];
    c4_prevpt[1] = c4_b_rtPts[28];
    for (c4_b_k = 0; c4_b_k < 27; c4_b_k++) {
      cudaMemcpy(chartInstance->c4_gpu_prevpt, &c4_prevpt[0], 8UL,
                 cudaMemcpyHostToDevice);
      c4_eML_blk_kernel_kernel120<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c4_gpu_prevpt, c4_b_k + 1, *chartInstance->c4_gpu_pts);
      if (c4_rtPts_dirtyOnCpu) {
        cudaMemcpy(chartInstance->c4_gpu_rtPts, &c4_b_rtPts[0], 224UL,
                   cudaMemcpyHostToDevice);
        c4_rtPts_dirtyOnCpu = false;
      }

      c4_eML_blk_kernel_kernel121<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
        (*chartInstance->c4_gpu_rtPts, c4_b_k, c4_b_k + 1,
         *chartInstance->c4_gpu_pts);
      c4_prevpt[0] = c4_b_rtPts[c4_b_k + 1];
      c4_prevpt[1] = c4_b_rtPts[c4_b_k + 29];
    }

    c4_eML_blk_kernel_kernel122<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_pts, *chartInstance->c4_gpu_positionOut);
    c4_pos.size[0] = 28;
    c4_pos.data[0].f1.size[0] = 4;
    cudaMemcpy(chartInstance->c4_gpu_pos, &c4_pos, 564UL, cudaMemcpyHostToDevice);
    c4_eML_blk_kernel_kernel123<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel124<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel125<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel126<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel127<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel128<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel129<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel130<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel131<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel132<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel133<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel134<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel135<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel136<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel137<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel138<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel139<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel140<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel141<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel142<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel143<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel144<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel145<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel146<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel147<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel148<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel149<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel150<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel151<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel152<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel153<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel154<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel155<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel156<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel157<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel158<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel159<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel160<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel161<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel162<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel163<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel164<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel165<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel166<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel167<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel168<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel169<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel170<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel171<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel172<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel173<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel174<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel175<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel176<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos);
    c4_eML_blk_kernel_kernel177<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_positionOut, chartInstance->c4_gpu_pos);
    for (c4_i127 = 0; c4_i127 < 112; c4_i127++) {
      if (c4_b_positionOut_data_dirtyOnGpu) {
        cudaMemcpy(&c4_b_positionOut_data[0],
                   chartInstance->c4_b_gpu_positionOut_data, 448UL,
                   cudaMemcpyDeviceToHost);
      }

      c4_b_positionOut_data[c4_i127] = 0;
      c4_b_positionOut_data_dirtyOnGpu = false;
    }

    c4_eML_blk_kernel_kernel178<<<dim3(1U, 1U, 1U), dim3(32U, 1U, 1U)>>>
      (chartInstance->c4_gpu_pos, *chartInstance->c4_b_gpu_positionOut_data);
    if (c4_In_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c4_b_gpu_In, (void *)c4_c_In, 3686400UL,
                 cudaMemcpyHostToDevice);
      c4_In_dirtyOnCpu = false;
    }

    c4_eML_blk_kernel_kernel179<<<dim3(1800U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_b_gpu_In, *chartInstance->c4_gpu_I);
    c4_I_dirtyOnGpu = true;
    c4_In_dirtyOnGpu = true;
    c4_eML_blk_kernel_kernel180<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_pixCount);
    c4_pixCount_dirtyOnGpu = true;
    c4_c_ptrObj = NULL;
    constructDrawBaseObjectShape(&c4_c_ptrObj);
    c4_eML_blk_kernel_kernel181<<<dim3(1U, 1U, 1U), dim3(128U, 1U, 1U)>>>
      (*chartInstance->c4_b_gpu_positionOut_data,
       *chartInstance->c4_gpu_positionOut_data);
    c4_c_posPtr = NULL;
    cudaMemcpy(&c4_positionOut_data[0], chartInstance->c4_gpu_positionOut_data,
               448UL, cudaMemcpyDeviceToHost);
    getPositionDataPointer(&c4_c_posPtr, &c4_positionOut_data[0], 28U, 4U);
    if (c4_color_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c4_gpu_color, &c4_color[0], 84UL,
                 cudaMemcpyHostToDevice);
    }

    c4_eML_blk_kernel_kernel182<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_color, *chartInstance->c4_b_gpu_color);
    c4_c_colPtr = NULL;
    cudaMemcpy(&c4_b_color[0], chartInstance->c4_b_gpu_color, 336UL,
               cudaMemcpyDeviceToHost);
    getColorDataPointer_single(&c4_c_colPtr, &c4_b_color[0], 28U, 3U);
    for (c4_f_i = 0; c4_f_i < 2; c4_f_i++) {
      c4_c_isInitialise = initialiseDrawbaseShape(c4_c_ptrObj, (int16_T)c4_f_i,
        2);
      if (!c4_c_isInitialise) {
        if (c4_In_dirtyOnGpu) {
          cudaMemcpy((void *)c4_c_In, chartInstance->c4_b_gpu_In, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_In_dirtyOnGpu = false;
        }

        if (c4_I_dirtyOnGpu) {
          cudaMemcpy(&chartInstance->c4_I[0], chartInstance->c4_gpu_I, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_I_dirtyOnGpu = false;
        }

        if (c4_pixCount_dirtyOnGpu) {
          cudaMemcpy(&c4_pixCount[0], chartInstance->c4_gpu_pixCount, 640UL,
                     cudaMemcpyDeviceToHost);
          c4_pixCount_dirtyOnGpu = false;
        }

        instantiateDrawBaseShape_single(c4_c_ptrObj, &c4_c_In[0],
          &chartInstance->c4_I[0], c4_c_posPtr, c4_c_colPtr, 0.6, 2, 2, true,
          480, 640, 3, 2, 28, 4, 28, false, c4_bv[c4_f_i], &c4_pixCount[0],
          (int16_T)c4_f_i);
        c4_In_dirtyOnCpu = true;
      }
    }

    mDrawShapes(c4_c_ptrObj, false, true, 2, 2, 480, 640);
    deallocateMemoryShape(c4_c_ptrObj);
    deletePositionDataPointer(c4_c_posPtr);
    deleteColorDataPointer_single(c4_c_colPtr);
    if (c4_In_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c4_b_gpu_In, (void *)c4_c_In, 3686400UL,
                 cudaMemcpyHostToDevice);
      c4_In_dirtyOnCpu = false;
    }

    c4_eML_blk_kernel_kernel183<<<dim3(1800U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_b_gpu_In, *chartInstance->c4_gpu_I);
    c4_I_dirtyOnGpu = true;
    if (c4_ltPts_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c4_gpu_ltPts, &c4_b_ltPts[0], 224UL,
                 cudaMemcpyHostToDevice);
    }

    c4_eML_blk_kernel_kernel184<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_ltPts, *chartInstance->c4_gpu_position);
    c4_position_dirtyOnGpu = true;
    cudaMemcpy(chartInstance->c4_gpu_fv1, &c4_fv1[0], 84UL,
               cudaMemcpyHostToDevice);
    c4_fv1_dirtyOnCpu = false;
    c4_eML_blk_kernel_kernel185<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_fv1, *chartInstance->c4_b_gpu_color);
    c4_color_dirtyOnGpu = true;
    c4_eML_blk_kernel_kernel186<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_pixCount);
    c4_pixCount_dirtyOnGpu = true;
    c4_d_ptrObj = NULL;
    constructDrawBaseObjectMarker(&c4_d_ptrObj);
    for (c4_g_i = 0; c4_g_i < 2; c4_g_i++) {
      c4_d_isInitialise = initialiseDrawbaseMarker(c4_d_ptrObj, (int16_T)c4_g_i);
      if (!c4_d_isInitialise) {
        if (c4_In_dirtyOnGpu) {
          cudaMemcpy((void *)c4_c_In, chartInstance->c4_b_gpu_In, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_In_dirtyOnGpu = false;
        }

        if (c4_I_dirtyOnGpu) {
          cudaMemcpy(&chartInstance->c4_I[0], chartInstance->c4_gpu_I, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_I_dirtyOnGpu = false;
        }

        if (c4_pixCount_dirtyOnGpu) {
          cudaMemcpy(&c4_pixCount[0], chartInstance->c4_gpu_pixCount, 640UL,
                     cudaMemcpyDeviceToHost);
          c4_pixCount_dirtyOnGpu = false;
        }

        if (c4_color_dirtyOnGpu) {
          cudaMemcpy(&c4_b_color[0], chartInstance->c4_b_gpu_color, 336UL,
                     cudaMemcpyDeviceToHost);
          c4_color_dirtyOnGpu = false;
        }

        if (c4_position_dirtyOnGpu) {
          cudaMemcpy(&c4_position[0], chartInstance->c4_gpu_position, 224UL,
                     cudaMemcpyDeviceToHost);
          c4_position_dirtyOnGpu = false;
        }

        instantiateDrawBaseMarker_single(c4_d_ptrObj, &c4_c_In[0],
          &chartInstance->c4_I[0], &c4_position[0], &c4_b_color[0], 0.6, 3, 3,
          480, 640, 3, 2, 28, 2, 28, &c4_pixCount[0], (int16_T)c4_g_i);
        c4_In_dirtyOnCpu = true;
      }
    }

    deallocateMemoryMarker(c4_d_ptrObj);
    if (c4_In_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c4_b_gpu_In, (void *)c4_c_In, 3686400UL,
                 cudaMemcpyHostToDevice);
    }

    c4_eML_blk_kernel_kernel187<<<dim3(1800U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_b_gpu_In, *chartInstance->c4_gpu_I);
    c4_I_dirtyOnGpu = true;
    if (c4_rtPts_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c4_gpu_rtPts, &c4_b_rtPts[0], 224UL,
                 cudaMemcpyHostToDevice);
    }

    c4_eML_blk_kernel_kernel188<<<dim3(1U, 1U, 1U), dim3(64U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_rtPts, *chartInstance->c4_gpu_position);
    c4_position_dirtyOnGpu = true;
    if (c4_fv1_dirtyOnCpu) {
      cudaMemcpy(chartInstance->c4_gpu_fv1, &c4_fv1[0], 84UL,
                 cudaMemcpyHostToDevice);
    }

    c4_eML_blk_kernel_kernel189<<<dim3(1U, 1U, 1U), dim3(96U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_fv1, *chartInstance->c4_b_gpu_color);
    c4_color_dirtyOnGpu = true;
    c4_eML_blk_kernel_kernel190<<<dim3(2U, 1U, 1U), dim3(512U, 1U, 1U)>>>
      (*chartInstance->c4_gpu_pixCount);
    c4_pixCount_dirtyOnGpu = true;
    c4_e_ptrObj = NULL;
    constructDrawBaseObjectMarker(&c4_e_ptrObj);
    for (c4_h_i = 0; c4_h_i < 2; c4_h_i++) {
      c4_e_isInitialise = initialiseDrawbaseMarker(c4_e_ptrObj, (int16_T)c4_h_i);
      if (!c4_e_isInitialise) {
        if (c4_In_dirtyOnGpu) {
          cudaMemcpy((void *)c4_c_In, chartInstance->c4_b_gpu_In, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_In_dirtyOnGpu = false;
        }

        if (c4_I_dirtyOnGpu) {
          cudaMemcpy(&chartInstance->c4_I[0], chartInstance->c4_gpu_I, 3686400UL,
                     cudaMemcpyDeviceToHost);
          c4_I_dirtyOnGpu = false;
        }

        if (c4_pixCount_dirtyOnGpu) {
          cudaMemcpy(&c4_pixCount[0], chartInstance->c4_gpu_pixCount, 640UL,
                     cudaMemcpyDeviceToHost);
          c4_pixCount_dirtyOnGpu = false;
        }

        if (c4_color_dirtyOnGpu) {
          cudaMemcpy(&c4_b_color[0], chartInstance->c4_b_gpu_color, 336UL,
                     cudaMemcpyDeviceToHost);
          c4_color_dirtyOnGpu = false;
        }

        if (c4_position_dirtyOnGpu) {
          cudaMemcpy(&c4_position[0], chartInstance->c4_gpu_position, 224UL,
                     cudaMemcpyDeviceToHost);
          c4_position_dirtyOnGpu = false;
        }

        instantiateDrawBaseMarker_single(c4_e_ptrObj, &c4_c_In[0],
          &chartInstance->c4_I[0], &c4_position[0], &c4_b_color[0], 0.6, 3, 3,
          480, 640, 3, 2, 28, 2, 28, &c4_pixCount[0], (int16_T)c4_h_i);
      }
    }

    deallocateMemoryMarker(c4_e_ptrObj);
  }

  if (c4_In_dirtyOnGpu) {
    cudaMemcpy((void *)c4_c_In, chartInstance->c4_b_gpu_In, 3686400UL,
               cudaMemcpyDeviceToHost);
  }
}

static __global__ __launch_bounds__(512, 1) void
  c4_sf_gateway_c4_LaneDetection_kernel1()
{
  int32_T c4_i;
  c4_i = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i < 921600) {
    c4_gpu_In[c4_i] = c4_gpu_In_1[c4_i];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel2(
  const real_T c4_b_bboxes_data[], const int32_T c4_bboxes_size[2], int32_T
  c4_position_data[80])
{
  int64_T c4_loopEnd;
  real_T c4_d;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i7;
  int32_T c4_i8;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_bboxes_size[0] * c4_bboxes_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i7 = (int32_T)c4_idx;
    c4_d = round(c4_b_bboxes_data[c4_i7]);
    if (c4_d < 2.147483648E+9) {
      if (c4_d >= -2.147483648E+9) {
        c4_i8 = (int32_T)c4_d;
      } else {
        c4_i8 = MIN_int32_T;
      }
    } else if (c4_d >= 2.147483648E+9) {
      c4_i8 = MAX_int32_T;
    } else {
      c4_i8 = 0;
    }

    c4_position_data[c4_i7] = c4_i8;
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel3(
  const int8_T c4_color_data[60], const int32_T c4_color_size[2], int8_T
  c4_b_color_data[60])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i13;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_color_size[0] * 3 - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i13 = (int32_T)c4_idx;
    c4_b_color_data[c4_i13] = c4_color_data[c4_i13];
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel4
  (real32_T c4_c_In[921600], real32_T c4_b_I[921600])
{
  int32_T c4_i14;
  c4_i14 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i14 < 921600) {
    c4_b_I[c4_i14] = c4_c_In[c4_i14];
    c4_c_In[c4_i14] = 0.0F;
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel5
  (uint8_T c4_pixCount[640])
{
  int32_T c4_i17;
  c4_i17 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i17 < 640) {
    c4_pixCount[c4_i17] = (uint8_T)0U;
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel6(
  const int32_T c4_position_data[80], const int32_T c4_position_size[2], int32_T
  c4_positionOut_data[112])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i19;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_position_size[0] * c4_position_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i19 = (int32_T)c4_idx;
    c4_positionOut_data[c4_i19] = c4_position_data[c4_i19];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel7(
  const int8_T c4_color_data[60], const int32_T c4_color_size[2], real32_T
  c4_b_color_data[60])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i22;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_color_size[0] * 3 - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i22 = (int32_T)c4_idx;
    c4_b_color_data[c4_i22] = (real32_T)c4_color_data[c4_i22];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel8(
  const int32_T c4_position_data[80], const int32_T c4_position_size[2], const
  int32_T c4_textLocAndWidth_size[2], const int32_T c4_i25, int32_T
  c4_textLocAndWidth_data[80])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i27;
  int32_T c4_i31;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = ((int64_T)c4_i25 + 1L) * 4L - 1L;
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i31 = (int32_T)(c4_idx % ((uint64_T)c4_i25 + 1UL));
    c4_i27 = (int32_T)((c4_idx - (uint64_T)c4_i31) / ((uint64_T)c4_i25 + 1UL));
    c4_textLocAndWidth_data[c4_i31 + c4_textLocAndWidth_size[0] * c4_i27] =
      c4_position_data[c4_i31 + c4_position_size[0] * c4_i27];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel9(
  const int32_T c4_textLocAndWidth_data[80], const int32_T
  c4_textLocAndWidth_size[2], const int32_T c4_i28, int32_T
  c4_b_textLocAndWidth_data[20])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i30;
  int32_T c4_q0;
  int32_T c4_qY;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)c4_i28;
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i30 = (int32_T)c4_idx;
    c4_q0 = c4_textLocAndWidth_data[c4_i30 + c4_textLocAndWidth_size[0]];
    if (c4_q0 < -2147483647) {
      c4_qY = MIN_int32_T;
    } else {
      c4_qY = c4_q0 - 1;
    }

    c4_b_textLocAndWidth_data[c4_i30] = c4_qY;
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel10(
  const int32_T c4_textLocAndWidth_data[20], const int32_T
  c4_textLocAndWidth_size[2], const int32_T c4_b_textLocAndWidth_size[1],
  int32_T c4_b_textLocAndWidth_data[80])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i32;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_b_textLocAndWidth_size[0] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i32 = (int32_T)c4_idx;
    c4_b_textLocAndWidth_data[c4_i32 + c4_textLocAndWidth_size[0]] =
      c4_textLocAndWidth_data[c4_i32];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel11(
  const int32_T c4_textLocAndWidth_data[80], const int32_T
  c4_textLocAndWidth_size[2], const int32_T c4_textPosition_size[2], const
  int32_T c4_i34, int32_T c4_textPosition_data[40])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i35;
  int32_T c4_i39;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = ((int64_T)c4_i34 + 1L) * 2L - 1L;
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i39 = (int32_T)(c4_idx % ((uint64_T)c4_i34 + 1UL));
    c4_i35 = (int32_T)((c4_idx - (uint64_T)c4_i39) / ((uint64_T)c4_i34 + 1UL));
    c4_textPosition_data[c4_i39 + c4_textPosition_size[0] * c4_i35] =
      c4_textLocAndWidth_data[c4_i39 + c4_textLocAndWidth_size[0] * c4_i35];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel12(
  const int8_T c4_color_data[60], const int32_T c4_color_size[2], int8_T
  c4_textColor_data[60])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i38;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_color_size[0] * 3 - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i38 = (int32_T)c4_idx;
    c4_textColor_data[c4_i38] = c4_color_data[c4_i38];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel13(
  const int32_T c4_textLocAndWidth_data[80], const int32_T
  c4_textLocAndWidth_size[2], const int32_T c4_iv[2], int32_T
  c4_shapeWidth_data[20])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i41;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_iv[0] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i41 = (int32_T)c4_idx;
    c4_shapeWidth_data[c4_i41] = c4_textLocAndWidth_data[c4_i41 +
      (c4_textLocAndWidth_size[0] << 1)];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel14(const
  int32_T c4_textLocAndWidth_data[80], int32_T c4_shapeWidth_data[20])
{
  int32_T c4_itilerow;
  c4_itilerow = (int32_T)mwGetGlobalThreadIndex();
  if (c4_itilerow < 1) {
    c4_shapeWidth_data[0] = c4_textLocAndWidth_data[2];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel15(
  const int32_T c4_textLocAndWidth_data[80], const int32_T
  c4_textLocAndWidth_size[2], const int32_T c4_iv1[2], int32_T
  c4_shapeHeight_data[20])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i44;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_iv1[0] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i44 = (int32_T)c4_idx;
    c4_shapeHeight_data[c4_i44] = c4_textLocAndWidth_data[c4_i44 +
      c4_textLocAndWidth_size[0] * 3];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel16(const
  int32_T c4_textLocAndWidth_data[80], int32_T c4_shapeHeight_data[20])
{
  int32_T c4_itilerow;
  c4_itilerow = (int32_T)mwGetGlobalThreadIndex();
  if (c4_itilerow < 1) {
    c4_shapeHeight_data[0] = c4_textLocAndWidth_data[3];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel17
  (char_T c4_str1[30])
{
  int32_T c4_i47;
  c4_i47 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i47 < 30) {
    c4_str1[c4_i47] = '\x00';
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel18(const
  char_T c4_cv8[6], char_T c4_cv7[6])
{
  int32_T c4_i48;
  c4_i48 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i48 < 6) {
    c4_cv7[c4_i48] = c4_cv8[c4_i48];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel19(
  const char_T c4_str1[30], const int32_T c4_i1, uint8_T c4_thisTextU16_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i52;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)c4_i1;
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i52 = (int32_T)c4_idx;
    c4_thisTextU16_data[c4_i52] = (uint8_T)c4_str1[c4_i52];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel20(
  const uint8_T c4_thisTextU16_data[29], const int32_T c4_thisTextU16_size[2],
  boolean_T c4_isNewLineChar_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i54;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisTextU16_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i54 = (int32_T)c4_idx;
    c4_isNewLineChar_data[c4_i54] = (boolean_T)((int32_T)
      c4_thisTextU16_data[c4_i54] == 10);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel21(
  const int8_T c4_ii_data[29], const int32_T c4_ii_size[2], int8_T
  c4_idxNewlineChar_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i58;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_ii_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i58 = (int32_T)c4_idx;
    c4_idxNewlineChar_data[c4_i58] = c4_ii_data[c4_i58];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel22(
  const uint8_T c4_thisTextU16_data[29], const int32_T c4_i3, uint16_T
  c4_thisCharcodes_1b_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i62;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)c4_i3;
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i62 = (int32_T)c4_idx;
    c4_thisCharcodes_1b_data[c4_i62] = (uint16_T)((uint32_T)(int32_T)
      c4_thisTextU16_data[c4_i62] + 1U);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel23(
  const int8_T c4_iv1[261], const uint16_T c4_uv[256], const uint16_T
  c4_thisCharcodes_1b_data[29], const int32_T c4_thisCharcodes_1b_size[2],
  int8_T c4_x_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i65;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisCharcodes_1b_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i65 = (int32_T)c4_idx;
    c4_x_data[c4_i65] = c4_iv1[c4_uv[(int32_T)c4_thisCharcodes_1b_data[c4_i65] -
      1]];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel24(const
  int8_T c4_iv1[261], const uint16_T c4_uv[256], const uint16_T
  c4_thisCharcodes_1b_data[29], real_T *c4_lenFirstSegment)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_lenFirstSegment = (real_T)c4_iv1[c4_uv[(int32_T)
      c4_thisCharcodes_1b_data[0] - 1]];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel25(
  const int32_T c4_x_size[2], const int8_T c4_x_data[29], int32_T c4_vlen,
  real_T *c4_lenFirstSegment)
{
  int64_T c4_loopEnd;
  real_T c4_tmpRed0;
  uint32_T c4_blockStride;
  uint32_T c4_idx;
  uint32_T c4_m;
  uint32_T c4_mask;
  uint32_T c4_numActiveThreads;
  uint32_T c4_numActiveWarps;
  uint32_T c4_thBlkId;
  uint32_T c4_threadId;
  uint32_T c4_threadStride;
  c4_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c4_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c4_tmpRed0 = 0.0;
  c4_loopEnd = (int64_T)(c4_vlen - 2);
  c4_numActiveThreads = c4_blockStride;
  if (mwIsLastBlock()) {
    c4_m = ((int64_T)(c4_vlen - 2) + 1L) % (int64_T)c4_blockStride;
    if (c4_m > 0U) {
      c4_numActiveThreads = c4_m;
    }
  }

  c4_numActiveWarps = (uint32_T)(c4_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c4_threadId <= c4_loopEnd) {
    c4_tmpRed0 = (real_T)c4_x_data[(int32_T)c4_threadId + 1];
  }

  c4_mask = __ballot_sync(MAX_uint32_T, (int64_T)c4_threadId <= c4_loopEnd);
  for (c4_idx = c4_threadId + c4_threadStride; c4_idx <= (uint32_T)c4_loopEnd;
       c4_idx += c4_threadStride) {
    c4_tmpRed0 += (real_T)c4_x_data[(int32_T)c4_idx + 1];
  }

  c4_tmpRed0 = c4_workGroupReduction(c4_tmpRed0, c4_mask, c4_numActiveWarps);
  if (c4_thBlkId == 0U) {
    c4_atomicOpreal_T(&c4_lenFirstSegment[0], c4_tmpRed0);
  }
}

static __device__ real_T c4_threadGroupReduction(real_T c4_val, uint32_T c4_lane,
  uint32_T c4_mask)
{
  real_T c4_other;
  uint32_T c4_activeSize;
  uint32_T c4_offset;
  c4_activeSize = __popc(c4_mask);
  c4_offset = (c4_activeSize + 1U) / 2U;
  while (c4_activeSize > 1U) {
    c4_other = c4_shflDown2(c4_val, c4_offset, c4_mask);
    if (c4_lane + c4_offset < c4_activeSize) {
      c4_val += c4_other;
    }

    c4_activeSize = c4_offset;
    c4_offset = (c4_offset + 1U) / 2U;
  }

  return c4_val;
}

static __device__ real_T c4_shflDown2(real_T c4_in1, uint32_T c4_offset,
  uint32_T c4_mask)
{
  int2 c4_tmp;
  c4_tmp = *(int2 *)&c4_in1;
  c4_tmp.x = __shfl_down_sync(c4_mask, c4_tmp.x, c4_offset);
  c4_tmp.y = __shfl_down_sync(c4_mask, c4_tmp.y, c4_offset);
  return *(real_T *)&c4_tmp;
}

static __device__ real_T c4_workGroupReduction(real_T c4_val, uint32_T c4_mask,
  uint32_T c4_numActiveWarps)
{
  __shared__ real_T c4_shared[32];
  uint32_T c4_lane;
  uint32_T c4_thBlkId;
  uint32_T c4_widx;
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_lane = c4_thBlkId % warpSize;
  c4_widx = c4_thBlkId / warpSize;
  c4_val = c4_threadGroupReduction(c4_val, c4_lane, c4_mask);
  if (c4_lane == 0U) {
    c4_shared[c4_widx] = c4_val;
  }

  __syncthreads();
  c4_mask = __ballot_sync(MAX_uint32_T, c4_lane < c4_numActiveWarps);
  c4_val = c4_shared[c4_lane];
  if (c4_widx == 0U) {
    c4_val = c4_threadGroupReduction(c4_val, c4_lane, c4_mask);
  }

  return c4_val;
}

static __device__ real_T c4_atomicOpreal_T(real_T *c4_address, real_T c4_value)
{
  unsigned long long int c4_assumed;
  unsigned long long int c4_old;
  unsigned long long int *c4_address_as_up;
  c4_address_as_up = (unsigned long long int *)c4_address;
  c4_old = *c4_address_as_up;
  do {
    c4_assumed = c4_old;
    c4_old = atomicCAS(c4_address_as_up, c4_old, __double_as_longlong(c4_value +
      __longlong_as_double(c4_old)));
  } while (c4_assumed != c4_old);

  return __longlong_as_double(c4_old);
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel26(
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], const
  int32_T c4_thisCharcodes_1b_size[2], boolean_T c4_isNewLineChar_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i68;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisCharcodes_1b_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i68 = (int32_T)c4_idx;
    c4_isNewLineChar_data[c4_i68] = (boolean_T)((int32_T)c4_uv[(int32_T)
      c4_thisCharcodes_1b_data[c4_i68] - 1] == 0);
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel27(const
  boolean_T c4_isNewLineChar_data[29], int32_T *c4_nz)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_nz = (int32_T)c4_isNewLineChar_data[0];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel28(
  const int32_T c4_isNewLineChar_size[2], const boolean_T c4_isNewLineChar_data
  [29], int32_T c4_vlen, int32_T *c4_nz)
{
  int64_T c4_loopEnd;
  int32_T c4_tmpRed0;
  uint32_T c4_blockStride;
  uint32_T c4_idx;
  uint32_T c4_m;
  uint32_T c4_mask;
  uint32_T c4_numActiveThreads;
  uint32_T c4_numActiveWarps;
  uint32_T c4_thBlkId;
  uint32_T c4_threadId;
  uint32_T c4_threadStride;
  c4_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c4_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c4_tmpRed0 = 0;
  c4_loopEnd = (int64_T)(c4_vlen - 2);
  c4_numActiveThreads = c4_blockStride;
  if (mwIsLastBlock()) {
    c4_m = ((int64_T)(c4_vlen - 2) + 1L) % (int64_T)c4_blockStride;
    if (c4_m > 0U) {
      c4_numActiveThreads = c4_m;
    }
  }

  c4_numActiveWarps = (uint32_T)(c4_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c4_threadId <= c4_loopEnd) {
    c4_tmpRed0 = (int32_T)c4_isNewLineChar_data[(int32_T)c4_threadId + 1];
  }

  c4_mask = __ballot_sync(MAX_uint32_T, (int64_T)c4_threadId <= c4_loopEnd);
  for (c4_idx = c4_threadId + c4_threadStride; c4_idx <= (uint32_T)c4_loopEnd;
       c4_idx += c4_threadStride) {
    c4_tmpRed0 += (int32_T)c4_isNewLineChar_data[(int32_T)c4_idx + 1];
  }

  c4_tmpRed0 = c4_b_workGroupReduction(c4_tmpRed0, c4_mask, c4_numActiveWarps);
  if (c4_thBlkId == 0U) {
    atomicAdd(&c4_nz[0], c4_tmpRed0);
  }
}

static __device__ int32_T c4_b_threadGroupReduction(int32_T c4_val, uint32_T
  c4_lane, uint32_T c4_mask)
{
  int32_T c4_other;
  uint32_T c4_activeSize;
  uint32_T c4_offset;
  c4_activeSize = __popc(c4_mask);
  c4_offset = (c4_activeSize + 1U) / 2U;
  while (c4_activeSize > 1U) {
    c4_other = c4_shflDown1(c4_val, c4_offset, c4_mask);
    if (c4_lane + c4_offset < c4_activeSize) {
      c4_val += c4_other;
    }

    c4_activeSize = c4_offset;
    c4_offset = (c4_offset + 1U) / 2U;
  }

  return c4_val;
}

static __device__ int32_T c4_shflDown1(int32_T c4_in1, uint32_T c4_offset,
  uint32_T c4_mask)
{
  c4_in1 = __shfl_down_sync(c4_mask, c4_in1, c4_offset);
  return c4_in1;
}

static __device__ int32_T c4_b_workGroupReduction(int32_T c4_val, uint32_T
  c4_mask, uint32_T c4_numActiveWarps)
{
  __shared__ int32_T c4_shared[32];
  uint32_T c4_lane;
  uint32_T c4_thBlkId;
  uint32_T c4_widx;
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_lane = c4_thBlkId % warpSize;
  c4_widx = c4_thBlkId / warpSize;
  c4_val = c4_b_threadGroupReduction(c4_val, c4_lane, c4_mask);
  if (c4_lane == 0U) {
    c4_shared[c4_widx] = c4_val;
  }

  __syncthreads();
  c4_mask = __ballot_sync(MAX_uint32_T, c4_lane < c4_numActiveWarps);
  c4_val = c4_shared[c4_lane];
  if (c4_widx == 0U) {
    c4_val = c4_b_threadGroupReduction(c4_val, c4_lane, c4_mask);
  }

  return c4_val;
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel29(
  const uint8_T c4_thisTextU16_data[29], const int32_T c4_i6, const int32_T
  c4_i8, uint16_T c4_thisCharcodes_1b_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i78;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_i8 - c4_i6);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i78 = (int32_T)c4_idx;
    c4_thisCharcodes_1b_data[c4_i78] = (uint16_T)((uint32_T)(int32_T)
      c4_thisTextU16_data[c4_i6 + c4_i78] + 1U);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel30(
  const int8_T c4_iv1[261], const uint16_T c4_uv[256], const uint16_T
  c4_thisCharcodes_1b_data[29], const int32_T c4_thisCharcodes_1b_size[2],
  int8_T c4_x_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i80;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisCharcodes_1b_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i80 = (int32_T)c4_idx;
    c4_x_data[c4_i80] = c4_iv1[c4_uv[(int32_T)c4_thisCharcodes_1b_data[c4_i80] -
      1]];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel31(const
  int8_T c4_iv1[261], const uint16_T c4_uv[256], const uint16_T
  c4_thisCharcodes_1b_data[29], real_T *c4_lenThisSegment)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_lenThisSegment = (real_T)c4_iv1[c4_uv[(int32_T)c4_thisCharcodes_1b_data
      [0] - 1]];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel32(
  const int32_T c4_x_size[2], const int8_T c4_x_data[29], int32_T c4_vlen,
  real_T *c4_lenThisSegment)
{
  int64_T c4_loopEnd;
  real_T c4_tmpRed0;
  uint32_T c4_blockStride;
  uint32_T c4_idx;
  uint32_T c4_m;
  uint32_T c4_mask;
  uint32_T c4_numActiveThreads;
  uint32_T c4_numActiveWarps;
  uint32_T c4_thBlkId;
  uint32_T c4_threadId;
  uint32_T c4_threadStride;
  c4_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c4_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c4_tmpRed0 = 0.0;
  c4_loopEnd = (int64_T)(c4_vlen - 2);
  c4_numActiveThreads = c4_blockStride;
  if (mwIsLastBlock()) {
    c4_m = ((int64_T)(c4_vlen - 2) + 1L) % (int64_T)c4_blockStride;
    if (c4_m > 0U) {
      c4_numActiveThreads = c4_m;
    }
  }

  c4_numActiveWarps = (uint32_T)(c4_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c4_threadId <= c4_loopEnd) {
    c4_tmpRed0 = (real_T)c4_x_data[(int32_T)c4_threadId + 1];
  }

  c4_mask = __ballot_sync(MAX_uint32_T, (int64_T)c4_threadId <= c4_loopEnd);
  for (c4_idx = c4_threadId + c4_threadStride; c4_idx <= (uint32_T)c4_loopEnd;
       c4_idx += c4_threadStride) {
    c4_tmpRed0 += (real_T)c4_x_data[(int32_T)c4_idx + 1];
  }

  c4_tmpRed0 = c4_workGroupReduction(c4_tmpRed0, c4_mask, c4_numActiveWarps);
  if (c4_thBlkId == 0U) {
    c4_atomicOpreal_T(&c4_lenThisSegment[0], c4_tmpRed0);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel33(
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], const
  int32_T c4_thisCharcodes_1b_size[2], boolean_T c4_isNewLineChar_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i82;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisCharcodes_1b_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i82 = (int32_T)c4_idx;
    c4_isNewLineChar_data[c4_i82] = (boolean_T)((int32_T)c4_uv[(int32_T)
      c4_thisCharcodes_1b_data[c4_i82] - 1] == 0);
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel34(const
  boolean_T c4_isNewLineChar_data[29], int32_T *c4_nz)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_nz = (int32_T)c4_isNewLineChar_data[0];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel35(
  const int32_T c4_isNewLineChar_size[2], const boolean_T c4_isNewLineChar_data
  [29], int32_T c4_vlen, int32_T *c4_nz)
{
  int64_T c4_loopEnd;
  int32_T c4_tmpRed0;
  uint32_T c4_blockStride;
  uint32_T c4_idx;
  uint32_T c4_m;
  uint32_T c4_mask;
  uint32_T c4_numActiveThreads;
  uint32_T c4_numActiveWarps;
  uint32_T c4_thBlkId;
  uint32_T c4_threadId;
  uint32_T c4_threadStride;
  c4_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c4_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c4_tmpRed0 = 0;
  c4_loopEnd = (int64_T)(c4_vlen - 2);
  c4_numActiveThreads = c4_blockStride;
  if (mwIsLastBlock()) {
    c4_m = ((int64_T)(c4_vlen - 2) + 1L) % (int64_T)c4_blockStride;
    if (c4_m > 0U) {
      c4_numActiveThreads = c4_m;
    }
  }

  c4_numActiveWarps = (uint32_T)(c4_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c4_threadId <= c4_loopEnd) {
    c4_tmpRed0 = (int32_T)c4_isNewLineChar_data[(int32_T)c4_threadId + 1];
  }

  c4_mask = __ballot_sync(MAX_uint32_T, (int64_T)c4_threadId <= c4_loopEnd);
  for (c4_idx = c4_threadId + c4_threadStride; c4_idx <= (uint32_T)c4_loopEnd;
       c4_idx += c4_threadStride) {
    c4_tmpRed0 += (int32_T)c4_isNewLineChar_data[(int32_T)c4_idx + 1];
  }

  c4_tmpRed0 = c4_b_workGroupReduction(c4_tmpRed0, c4_mask, c4_numActiveWarps);
  if (c4_thBlkId == 0U) {
    atomicAdd(&c4_nz[0], c4_tmpRed0);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel36(
  const uint8_T c4_thisTextU16_data[29], const int32_T c4_i5, const int32_T
  c4_i7, uint16_T c4_thisCharcodes_1b_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i77;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_i7 - c4_i5);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i77 = (int32_T)c4_idx;
    c4_thisCharcodes_1b_data[c4_i77] = (uint16_T)((uint32_T)(int32_T)
      c4_thisTextU16_data[c4_i5 + c4_i77] + 1U);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel37(
  const int8_T c4_iv1[261], const uint16_T c4_uv[256], const uint16_T
  c4_thisCharcodes_1b_data[29], const int32_T c4_thisCharcodes_1b_size[2],
  int8_T c4_x_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i79;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisCharcodes_1b_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i79 = (int32_T)c4_idx;
    c4_x_data[c4_i79] = c4_iv1[c4_uv[(int32_T)c4_thisCharcodes_1b_data[c4_i79] -
      1]];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel38(const
  int8_T c4_iv1[261], const uint16_T c4_uv[256], const uint16_T
  c4_thisCharcodes_1b_data[29], real_T *c4_lenEndSegment)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_lenEndSegment = (real_T)c4_iv1[c4_uv[(int32_T)c4_thisCharcodes_1b_data[0]
      - 1]];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel39(
  const int32_T c4_x_size[2], const int8_T c4_x_data[29], int32_T c4_vlen,
  real_T *c4_lenEndSegment)
{
  int64_T c4_loopEnd;
  real_T c4_tmpRed0;
  uint32_T c4_blockStride;
  uint32_T c4_idx;
  uint32_T c4_m;
  uint32_T c4_mask;
  uint32_T c4_numActiveThreads;
  uint32_T c4_numActiveWarps;
  uint32_T c4_thBlkId;
  uint32_T c4_threadId;
  uint32_T c4_threadStride;
  c4_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c4_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c4_tmpRed0 = 0.0;
  c4_loopEnd = (int64_T)(c4_vlen - 2);
  c4_numActiveThreads = c4_blockStride;
  if (mwIsLastBlock()) {
    c4_m = ((int64_T)(c4_vlen - 2) + 1L) % (int64_T)c4_blockStride;
    if (c4_m > 0U) {
      c4_numActiveThreads = c4_m;
    }
  }

  c4_numActiveWarps = (uint32_T)(c4_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c4_threadId <= c4_loopEnd) {
    c4_tmpRed0 = (real_T)c4_x_data[(int32_T)c4_threadId + 1];
  }

  c4_mask = __ballot_sync(MAX_uint32_T, (int64_T)c4_threadId <= c4_loopEnd);
  for (c4_idx = c4_threadId + c4_threadStride; c4_idx <= (uint32_T)c4_loopEnd;
       c4_idx += c4_threadStride) {
    c4_tmpRed0 += (real_T)c4_x_data[(int32_T)c4_idx + 1];
  }

  c4_tmpRed0 = c4_workGroupReduction(c4_tmpRed0, c4_mask, c4_numActiveWarps);
  if (c4_thBlkId == 0U) {
    c4_atomicOpreal_T(&c4_lenEndSegment[0], c4_tmpRed0);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel40(
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], const
  int32_T c4_thisCharcodes_1b_size[2], boolean_T c4_isNewLineChar_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i81;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisCharcodes_1b_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i81 = (int32_T)c4_idx;
    c4_isNewLineChar_data[c4_i81] = (boolean_T)((int32_T)c4_uv[(int32_T)
      c4_thisCharcodes_1b_data[c4_i81] - 1] == 0);
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel41(const
  boolean_T c4_isNewLineChar_data[29], int32_T *c4_nz)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_nz = (int32_T)c4_isNewLineChar_data[0];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel42(
  const int32_T c4_isNewLineChar_size[2], const boolean_T c4_isNewLineChar_data
  [29], int32_T c4_vlen, int32_T *c4_nz)
{
  int64_T c4_loopEnd;
  int32_T c4_tmpRed0;
  uint32_T c4_blockStride;
  uint32_T c4_idx;
  uint32_T c4_m;
  uint32_T c4_mask;
  uint32_T c4_numActiveThreads;
  uint32_T c4_numActiveWarps;
  uint32_T c4_thBlkId;
  uint32_T c4_threadId;
  uint32_T c4_threadStride;
  c4_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c4_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c4_tmpRed0 = 0;
  c4_loopEnd = (int64_T)(c4_vlen - 2);
  c4_numActiveThreads = c4_blockStride;
  if (mwIsLastBlock()) {
    c4_m = ((int64_T)(c4_vlen - 2) + 1L) % (int64_T)c4_blockStride;
    if (c4_m > 0U) {
      c4_numActiveThreads = c4_m;
    }
  }

  c4_numActiveWarps = (uint32_T)(c4_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c4_threadId <= c4_loopEnd) {
    c4_tmpRed0 = (int32_T)c4_isNewLineChar_data[(int32_T)c4_threadId + 1];
  }

  c4_mask = __ballot_sync(MAX_uint32_T, (int64_T)c4_threadId <= c4_loopEnd);
  for (c4_idx = c4_threadId + c4_threadStride; c4_idx <= (uint32_T)c4_loopEnd;
       c4_idx += c4_threadStride) {
    c4_tmpRed0 += (int32_T)c4_isNewLineChar_data[(int32_T)c4_idx + 1];
  }

  c4_tmpRed0 = c4_b_workGroupReduction(c4_tmpRed0, c4_mask, c4_numActiveWarps);
  if (c4_thBlkId == 0U) {
    atomicAdd(&c4_nz[0], c4_tmpRed0);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel43(
  const uint8_T c4_thisTextU16_data[29], const int32_T c4_thisTextU16_size[2],
  uint16_T c4_thisCharcodes_1b_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i61;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisTextU16_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i61 = (int32_T)c4_idx;
    c4_thisCharcodes_1b_data[c4_i61] = (uint16_T)((uint32_T)(int32_T)
      c4_thisTextU16_data[c4_i61] + 1U);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel44(
  const int8_T c4_iv1[261], const uint16_T c4_uv[256], const uint16_T
  c4_thisCharcodes_1b_data[29], const int32_T c4_thisCharcodes_1b_size[2],
  int8_T c4_x_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i64;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisCharcodes_1b_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i64 = (int32_T)c4_idx;
    c4_x_data[c4_i64] = c4_iv1[c4_uv[(int32_T)c4_thisCharcodes_1b_data[c4_i64] -
      1]];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel45(const
  int8_T c4_iv1[261], const uint16_T c4_uv[256], const uint16_T
  c4_thisCharcodes_1b_data[29], real_T *c4_y)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_y = (real_T)c4_iv1[c4_uv[(int32_T)c4_thisCharcodes_1b_data[0] - 1]];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel46(
  const int32_T c4_x_size[2], const int8_T c4_x_data[29], int32_T c4_vlen,
  real_T *c4_y)
{
  int64_T c4_loopEnd;
  real_T c4_tmpRed0;
  uint32_T c4_blockStride;
  uint32_T c4_idx;
  uint32_T c4_m;
  uint32_T c4_mask;
  uint32_T c4_numActiveThreads;
  uint32_T c4_numActiveWarps;
  uint32_T c4_thBlkId;
  uint32_T c4_threadId;
  uint32_T c4_threadStride;
  c4_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c4_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c4_tmpRed0 = 0.0;
  c4_loopEnd = (int64_T)(c4_vlen - 2);
  c4_numActiveThreads = c4_blockStride;
  if (mwIsLastBlock()) {
    c4_m = ((int64_T)(c4_vlen - 2) + 1L) % (int64_T)c4_blockStride;
    if (c4_m > 0U) {
      c4_numActiveThreads = c4_m;
    }
  }

  c4_numActiveWarps = (uint32_T)(c4_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c4_threadId <= c4_loopEnd) {
    c4_tmpRed0 = (real_T)c4_x_data[(int32_T)c4_threadId + 1];
  }

  c4_mask = __ballot_sync(MAX_uint32_T, (int64_T)c4_threadId <= c4_loopEnd);
  for (c4_idx = c4_threadId + c4_threadStride; c4_idx <= (uint32_T)c4_loopEnd;
       c4_idx += c4_threadStride) {
    c4_tmpRed0 += (real_T)c4_x_data[(int32_T)c4_idx + 1];
  }

  c4_tmpRed0 = c4_workGroupReduction(c4_tmpRed0, c4_mask, c4_numActiveWarps);
  if (c4_thBlkId == 0U) {
    c4_atomicOpreal_T(&c4_y[0], c4_tmpRed0);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel47(
  const uint16_T c4_uv[256], const uint16_T c4_thisCharcodes_1b_data[29], const
  int32_T c4_thisCharcodes_1b_size[2], boolean_T c4_isNewLineChar_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i67;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisCharcodes_1b_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i67 = (int32_T)c4_idx;
    c4_isNewLineChar_data[c4_i67] = (boolean_T)((int32_T)c4_uv[(int32_T)
      c4_thisCharcodes_1b_data[c4_i67] - 1] == 0);
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel48(const
  boolean_T c4_isNewLineChar_data[29], int32_T *c4_nz)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_nz = (int32_T)c4_isNewLineChar_data[0];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel49(
  const int32_T c4_isNewLineChar_size[2], const boolean_T c4_isNewLineChar_data
  [29], int32_T c4_vlen, int32_T *c4_nz)
{
  int64_T c4_loopEnd;
  int32_T c4_tmpRed0;
  uint32_T c4_blockStride;
  uint32_T c4_idx;
  uint32_T c4_m;
  uint32_T c4_mask;
  uint32_T c4_numActiveThreads;
  uint32_T c4_numActiveWarps;
  uint32_T c4_thBlkId;
  uint32_T c4_threadId;
  uint32_T c4_threadStride;
  c4_threadStride = (uint32_T)mwGetTotalThreadsLaunched();
  c4_threadId = (uint32_T)mwGetGlobalThreadIndex();
  c4_thBlkId = (uint32_T)mwGetThreadIndexWithinBlock();
  c4_blockStride = (uint32_T)mwGetThreadsPerBlock();
  c4_tmpRed0 = 0;
  c4_loopEnd = (int64_T)(c4_vlen - 2);
  c4_numActiveThreads = c4_blockStride;
  if (mwIsLastBlock()) {
    c4_m = ((int64_T)(c4_vlen - 2) + 1L) % (int64_T)c4_blockStride;
    if (c4_m > 0U) {
      c4_numActiveThreads = c4_m;
    }
  }

  c4_numActiveWarps = (uint32_T)(c4_numActiveThreads + ((int64_T)warpSize - 1L))
    / warpSize;
  if ((int64_T)c4_threadId <= c4_loopEnd) {
    c4_tmpRed0 = (int32_T)c4_isNewLineChar_data[(int32_T)c4_threadId + 1];
  }

  c4_mask = __ballot_sync(MAX_uint32_T, (int64_T)c4_threadId <= c4_loopEnd);
  for (c4_idx = c4_threadId + c4_threadStride; c4_idx <= (uint32_T)c4_loopEnd;
       c4_idx += c4_threadStride) {
    c4_tmpRed0 += (int32_T)c4_isNewLineChar_data[(int32_T)c4_idx + 1];
  }

  c4_tmpRed0 = c4_b_workGroupReduction(c4_tmpRed0, c4_mask, c4_numActiveWarps);
  if (c4_thBlkId == 0U) {
    atomicAdd(&c4_nz[0], c4_tmpRed0);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel50(
  const uint8_T c4_thisTextU16_data[29], const int32_T c4_thisTextU16_size[2],
  boolean_T c4_isNewLineChar_data[29])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i101;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_thisTextU16_size[1] - 1);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i101 = (int32_T)c4_idx;
    c4_isNewLineChar_data[c4_i101] = (boolean_T)((int32_T)
      c4_thisTextU16_data[c4_i101] == 10);
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel51(const
  uint16_T c4_uv[256], const int32_T c4_i, const uint8_T c4_thisTextU16_data[29],
  uint16_T *c4_thisGlyphIdx_1b)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_thisGlyphIdx_1b = (uint16_T)((int32_T)c4_uv[c4_thisTextU16_data[c4_i]] +
      1);
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel52(
  const uint8_T c4_uv4[10664], const int32_T c4_i10, const int32_T c4_i11,
  uint8_T c4_uv4_data[10664])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i116;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = (int64_T)(c4_i11 - c4_i10);
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i116 = (int32_T)c4_idx;
    c4_uv4_data[c4_i116] = c4_uv4[c4_i10 + c4_i116];
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel53(
  const uint8_T c4_uv4_data[10664], const int32_T c4_uv4_size[2], const int8_T
  c4_outsize[2], uint8_T c4_b_uv4_data[144])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  uint64_T c4_tmpIndex;
  int32_T c4_i119;
  int32_T c4_i121;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = ((int64_T)((int32_T)c4_outsize[1] - 1) + 1L) * ((int64_T)
    ((int32_T)c4_outsize[0] - 1) + 1L) - 1L;
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i121 = (int32_T)(c4_idx % ((uint64_T)((int32_T)c4_outsize[1] - 1) + 1UL));
    c4_tmpIndex = (c4_idx - (uint64_T)c4_i121) / ((uint64_T)((int32_T)
      c4_outsize[1] - 1) + 1UL);
    c4_i119 = (int32_T)c4_tmpIndex;
    c4_b_uv4_data[c4_i121 + c4_uv4_size[0] * c4_i119] = c4_uv4_data[c4_i119 +
      (int32_T)c4_outsize[0] * c4_i121];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel54(const
  int8_T c4_outsize_idx_1, int8_T c4_outsize[2])
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_outsize[0] = c4_outsize_idx_1;
  }
}

static __global__ __launch_bounds__(1024, 1) void c4_eML_blk_kernel_kernel55(
  const uint8_T c4_uv4_data[144], const int8_T c4_outsize[2], const int32_T
  c4_thisGlyphCut_float_size[2], const int32_T c4_i12, const int32_T c4_i13,
  const int32_T c4_i14, const int32_T c4_i15, real_T c4_thisGlyphCut_float_data
  [132])
{
  int64_T c4_loopEnd;
  uint64_T c4_idx;
  uint64_T c4_threadId;
  uint64_T c4_threadStride;
  int32_T c4_i122;
  int32_T c4_i125;
  c4_threadId = mwGetGlobalThreadIndex();
  c4_threadStride = mwGetTotalThreadsLaunched();
  c4_loopEnd = ((int64_T)(c4_i13 - c4_i12) + 1L) * ((int64_T)(c4_i15 - c4_i14) +
    1L) - 1L;
  for (c4_idx = c4_threadId; c4_idx <= (uint64_T)c4_loopEnd; c4_idx +=
       c4_threadStride) {
    c4_i125 = (int32_T)(c4_idx % ((uint64_T)(c4_i13 - c4_i12) + 1UL));
    c4_i122 = (int32_T)((c4_idx - (uint64_T)c4_i125) / ((uint64_T)(c4_i13 -
      c4_i12) + 1UL));
    c4_thisGlyphCut_float_data[c4_i125 + c4_thisGlyphCut_float_size[0] * c4_i122]
      = (real_T)c4_uv4_data[(c4_i12 + c4_i125) + (int32_T)c4_outsize[0] *
      (c4_i14 + c4_i122)] / 255.0;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel56(const
  int8_T c4_iv1[261], const uint16_T *c4_thisGlyphIdx_1b, int32_T *c4_q1)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    *c4_q1 = (int32_T)c4_iv1[(int32_T)*c4_thisGlyphIdx_1b - 1];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel57(const
  real32_T c4_prevpt[2], const int32_T c4_k, real32_T c4_pts[112])
{
  int32_T c4_i10;
  c4_i10 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i10 < 2) {
    c4_pts[c4_k + 28 * c4_i10] = c4_prevpt[c4_i10];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel58(const
  real32_T c4_b_ltPts[56], const int32_T c4_k, const int32_T c4_b_k, real32_T
  c4_pts[112])
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pts[c4_b_k + 56] = c4_b_ltPts[c4_k + 1];
    c4_pts[c4_b_k + 84] = c4_b_ltPts[c4_k + 29];
  }
}

static __global__ __launch_bounds__(128, 1) void c4_eML_blk_kernel_kernel59(
  const real32_T c4_pts[112], int32_T c4_positionOut[112])
{
  int32_T c4_i11;
  int32_T c4_i9;
  real32_T c4_f;
  c4_i9 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i9 < 112) {
    c4_f = roundf(c4_pts[c4_i9]);
    if (c4_f < 2.14748365E+9F) {
      if (c4_f >= -2.14748365E+9F) {
        c4_i11 = (int32_T)c4_f;
      } else {
        c4_i11 = MIN_int32_T;
      }
    } else if (c4_f >= 2.14748365E+9F) {
      c4_i11 = MAX_int32_T;
    } else {
      c4_i11 = 0;
    }

    c4_positionOut[c4_i9] = c4_i11;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel60(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i12;
  c4_i12 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i12 < 4) {
    c4_pos->data[0].f1.data[c4_i12] = c4_positionOut[28 * c4_i12];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel61
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[1].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel62(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i15;
  c4_i15 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i15 < 4) {
    c4_pos->data[1].f1.data[c4_i15] = c4_positionOut[28 * c4_i15 + 1];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel63
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[2].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel64(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i16;
  c4_i16 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i16 < 4) {
    c4_pos->data[2].f1.data[c4_i16] = c4_positionOut[28 * c4_i16 + 2];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel65
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[3].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel66(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i18;
  c4_i18 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i18 < 4) {
    c4_pos->data[3].f1.data[c4_i18] = c4_positionOut[28 * c4_i18 + 3];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel67
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[4].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel68(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i20;
  c4_i20 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i20 < 4) {
    c4_pos->data[4].f1.data[c4_i20] = c4_positionOut[28 * c4_i20 + 4];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel69
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[5].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel70(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i21;
  c4_i21 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i21 < 4) {
    c4_pos->data[5].f1.data[c4_i21] = c4_positionOut[28 * c4_i21 + 5];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel71
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[6].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel72(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i23;
  c4_i23 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i23 < 4) {
    c4_pos->data[6].f1.data[c4_i23] = c4_positionOut[28 * c4_i23 + 6];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel73
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[7].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel74(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i24;
  c4_i24 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i24 < 4) {
    c4_pos->data[7].f1.data[c4_i24] = c4_positionOut[28 * c4_i24 + 7];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel75
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[8].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel76(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i26;
  c4_i26 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i26 < 4) {
    c4_pos->data[8].f1.data[c4_i26] = c4_positionOut[28 * c4_i26 + 8];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel77
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[9].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel78(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i29;
  c4_i29 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i29 < 4) {
    c4_pos->data[9].f1.data[c4_i29] = c4_positionOut[28 * c4_i29 + 9];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel79
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[10].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel80(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i33;
  c4_i33 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i33 < 4) {
    c4_pos->data[10].f1.data[c4_i33] = c4_positionOut[28 * c4_i33 + 10];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel81
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[11].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel82(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i36;
  c4_i36 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i36 < 4) {
    c4_pos->data[11].f1.data[c4_i36] = c4_positionOut[28 * c4_i36 + 11];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel83
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[12].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel84(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i37;
  c4_i37 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i37 < 4) {
    c4_pos->data[12].f1.data[c4_i37] = c4_positionOut[28 * c4_i37 + 12];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel85
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[13].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel86(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i40;
  c4_i40 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i40 < 4) {
    c4_pos->data[13].f1.data[c4_i40] = c4_positionOut[28 * c4_i40 + 13];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel87
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[14].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel88(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i42;
  c4_i42 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i42 < 4) {
    c4_pos->data[14].f1.data[c4_i42] = c4_positionOut[28 * c4_i42 + 14];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel89
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[15].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel90(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i43;
  c4_i43 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i43 < 4) {
    c4_pos->data[15].f1.data[c4_i43] = c4_positionOut[28 * c4_i43 + 15];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel91
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[16].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel92(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i45;
  c4_i45 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i45 < 4) {
    c4_pos->data[16].f1.data[c4_i45] = c4_positionOut[28 * c4_i45 + 16];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel93
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[17].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel94(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i46;
  c4_i46 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i46 < 4) {
    c4_pos->data[17].f1.data[c4_i46] = c4_positionOut[28 * c4_i46 + 17];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel95
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[18].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel96(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i49;
  c4_i49 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i49 < 4) {
    c4_pos->data[18].f1.data[c4_i49] = c4_positionOut[28 * c4_i49 + 18];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel97
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[19].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel98(const
  int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i50;
  c4_i50 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i50 < 4) {
    c4_pos->data[19].f1.data[c4_i50] = c4_positionOut[28 * c4_i50 + 19];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel99
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[20].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel100(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i51;
  c4_i51 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i51 < 4) {
    c4_pos->data[20].f1.data[c4_i51] = c4_positionOut[28 * c4_i51 + 20];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel101
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[21].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel102(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i53;
  c4_i53 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i53 < 4) {
    c4_pos->data[21].f1.data[c4_i53] = c4_positionOut[28 * c4_i53 + 21];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel103
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[22].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel104(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i55;
  c4_i55 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i55 < 4) {
    c4_pos->data[22].f1.data[c4_i55] = c4_positionOut[28 * c4_i55 + 22];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel105
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[23].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel106(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i56;
  c4_i56 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i56 < 4) {
    c4_pos->data[23].f1.data[c4_i56] = c4_positionOut[28 * c4_i56 + 23];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel107
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[24].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel108(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i57;
  c4_i57 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i57 < 4) {
    c4_pos->data[24].f1.data[c4_i57] = c4_positionOut[28 * c4_i57 + 24];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel109
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[25].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel110(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i59;
  c4_i59 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i59 < 4) {
    c4_pos->data[25].f1.data[c4_i59] = c4_positionOut[28 * c4_i59 + 25];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel111
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[26].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel112(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i60;
  c4_i60 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i60 < 4) {
    c4_pos->data[26].f1.data[c4_i60] = c4_positionOut[28 * c4_i60 + 26];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel113
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[27].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel114(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i63;
  c4_i63 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i63 < 4) {
    c4_pos->data[27].f1.data[c4_i63] = c4_positionOut[28 * c4_i63 + 27];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel115(
  const c4_emxArray_cell_wrap_21_28 *c4_pos, int32_T c4_positionOut_data[112])
{
  int32_T c4_out[4];
  int32_T c4_i70;
  int32_T c4_i71;
  int32_T c4_ii;
  c4_ii = (int32_T)mwGetGlobalThreadIndex();
  if (c4_ii < 28) {
    for (c4_i71 = 0; c4_i71 < 4; c4_i71++) {
      c4_out[c4_i71] = c4_pos->data[c4_ii].f1.data[c4_i71];
    }

    for (c4_i70 = 0; c4_i70 < 4; c4_i70++) {
      c4_positionOut_data[c4_ii + 28 * c4_i70] = c4_out[c4_i70];
    }
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel116
  (real32_T c4_c_In[921600], real32_T c4_b_I[921600])
{
  int32_T c4_i69;
  c4_i69 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i69 < 921600) {
    c4_b_I[c4_i69] = c4_c_In[c4_i69];
    c4_c_In[c4_i69] = 0.0F;
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel117
  (uint8_T c4_pixCount[640])
{
  int32_T c4_i72;
  c4_i72 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i72 < 640) {
    c4_pixCount[c4_i72] = (uint8_T)0U;
  }
}

static __global__ __launch_bounds__(128, 1) void c4_eML_blk_kernel_kernel118(
  const int32_T c4_positionOut_data[112], int32_T c4_b_positionOut_data[112])
{
  int32_T c4_i74;
  c4_i74 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i74 < 112) {
    c4_b_positionOut_data[c4_i74] = c4_positionOut_data[c4_i74];
  }
}

static __global__ __launch_bounds__(96, 1) void c4_eML_blk_kernel_kernel119(
  const int8_T c4_color[84], real32_T c4_b_color[84])
{
  int32_T c4_i76;
  c4_i76 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i76 < 84) {
    c4_b_color[c4_i76] = (real32_T)c4_color[c4_i76];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel120(
  const real32_T c4_prevpt[2], const int32_T c4_k, real32_T c4_pts[112])
{
  int32_T c4_i84;
  c4_i84 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i84 < 2) {
    c4_pts[c4_k + 28 * c4_i84] = c4_prevpt[c4_i84];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel121(
  const real32_T c4_b_rtPts[56], const int32_T c4_k, const int32_T c4_b_k,
  real32_T c4_pts[112])
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pts[c4_b_k + 56] = c4_b_rtPts[c4_k + 1];
    c4_pts[c4_b_k + 84] = c4_b_rtPts[c4_k + 29];
  }
}

static __global__ __launch_bounds__(128, 1) void c4_eML_blk_kernel_kernel122(
  const real32_T c4_pts[112], int32_T c4_positionOut[112])
{
  int32_T c4_i83;
  int32_T c4_i85;
  real32_T c4_f1;
  c4_i83 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i83 < 112) {
    c4_f1 = roundf(c4_pts[c4_i83]);
    if (c4_f1 < 2.14748365E+9F) {
      if (c4_f1 >= -2.14748365E+9F) {
        c4_i85 = (int32_T)c4_f1;
      } else {
        c4_i85 = MIN_int32_T;
      }
    } else if (c4_f1 >= 2.14748365E+9F) {
      c4_i85 = MAX_int32_T;
    } else {
      c4_i85 = 0;
    }

    c4_positionOut[c4_i83] = c4_i85;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel123(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i86;
  c4_i86 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i86 < 4) {
    c4_pos->data[0].f1.data[c4_i86] = c4_positionOut[28 * c4_i86];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel124
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[1].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel125(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i89;
  c4_i89 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i89 < 4) {
    c4_pos->data[1].f1.data[c4_i89] = c4_positionOut[28 * c4_i89 + 1];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel126
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[2].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel127(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i92;
  c4_i92 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i92 < 4) {
    c4_pos->data[2].f1.data[c4_i92] = c4_positionOut[28 * c4_i92 + 2];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel128
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[3].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel129(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i94;
  c4_i94 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i94 < 4) {
    c4_pos->data[3].f1.data[c4_i94] = c4_positionOut[28 * c4_i94 + 3];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel130
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[4].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel131(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i96;
  c4_i96 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i96 < 4) {
    c4_pos->data[4].f1.data[c4_i96] = c4_positionOut[28 * c4_i96 + 4];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel132
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[5].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel133(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i97;
  c4_i97 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i97 < 4) {
    c4_pos->data[5].f1.data[c4_i97] = c4_positionOut[28 * c4_i97 + 5];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel134
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[6].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel135(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i98;
  c4_i98 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i98 < 4) {
    c4_pos->data[6].f1.data[c4_i98] = c4_positionOut[28 * c4_i98 + 6];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel136
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[7].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel137(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i99;
  c4_i99 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i99 < 4) {
    c4_pos->data[7].f1.data[c4_i99] = c4_positionOut[28 * c4_i99 + 7];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel138
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[8].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel139(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i100;
  c4_i100 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i100 < 4) {
    c4_pos->data[8].f1.data[c4_i100] = c4_positionOut[28 * c4_i100 + 8];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel140
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[9].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel141(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i103;
  c4_i103 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i103 < 4) {
    c4_pos->data[9].f1.data[c4_i103] = c4_positionOut[28 * c4_i103 + 9];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel142
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[10].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel143(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i104;
  c4_i104 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i104 < 4) {
    c4_pos->data[10].f1.data[c4_i104] = c4_positionOut[28 * c4_i104 + 10];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel144
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[11].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel145(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i105;
  c4_i105 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i105 < 4) {
    c4_pos->data[11].f1.data[c4_i105] = c4_positionOut[28 * c4_i105 + 11];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel146
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[12].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel147(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i106;
  c4_i106 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i106 < 4) {
    c4_pos->data[12].f1.data[c4_i106] = c4_positionOut[28 * c4_i106 + 12];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel148
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[13].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel149(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i107;
  c4_i107 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i107 < 4) {
    c4_pos->data[13].f1.data[c4_i107] = c4_positionOut[28 * c4_i107 + 13];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel150
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[14].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel151(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i108;
  c4_i108 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i108 < 4) {
    c4_pos->data[14].f1.data[c4_i108] = c4_positionOut[28 * c4_i108 + 14];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel152
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[15].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel153(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i109;
  c4_i109 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i109 < 4) {
    c4_pos->data[15].f1.data[c4_i109] = c4_positionOut[28 * c4_i109 + 15];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel154
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[16].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel155(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i110;
  c4_i110 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i110 < 4) {
    c4_pos->data[16].f1.data[c4_i110] = c4_positionOut[28 * c4_i110 + 16];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel156
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[17].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel157(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i111;
  c4_i111 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i111 < 4) {
    c4_pos->data[17].f1.data[c4_i111] = c4_positionOut[28 * c4_i111 + 17];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel158
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[18].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel159(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i112;
  c4_i112 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i112 < 4) {
    c4_pos->data[18].f1.data[c4_i112] = c4_positionOut[28 * c4_i112 + 18];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel160
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[19].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel161(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i113;
  c4_i113 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i113 < 4) {
    c4_pos->data[19].f1.data[c4_i113] = c4_positionOut[28 * c4_i113 + 19];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel162
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[20].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel163(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i114;
  c4_i114 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i114 < 4) {
    c4_pos->data[20].f1.data[c4_i114] = c4_positionOut[28 * c4_i114 + 20];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel164
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[21].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel165(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i115;
  c4_i115 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i115 < 4) {
    c4_pos->data[21].f1.data[c4_i115] = c4_positionOut[28 * c4_i115 + 21];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel166
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[22].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel167(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i117;
  c4_i117 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i117 < 4) {
    c4_pos->data[22].f1.data[c4_i117] = c4_positionOut[28 * c4_i117 + 22];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel168
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[23].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel169(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i118;
  c4_i118 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i118 < 4) {
    c4_pos->data[23].f1.data[c4_i118] = c4_positionOut[28 * c4_i118 + 23];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel170
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[24].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel171(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i120;
  c4_i120 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i120 < 4) {
    c4_pos->data[24].f1.data[c4_i120] = c4_positionOut[28 * c4_i120 + 24];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel172
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[25].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel173(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i123;
  c4_i123 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i123 < 4) {
    c4_pos->data[25].f1.data[c4_i123] = c4_positionOut[28 * c4_i123 + 25];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel174
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[26].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel175(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i124;
  c4_i124 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i124 < 4) {
    c4_pos->data[26].f1.data[c4_i124] = c4_positionOut[28 * c4_i124 + 26];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel176
  (c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_tmpIdx;
  c4_tmpIdx = (int32_T)mwGetGlobalThreadIndex();
  if (c4_tmpIdx < 1) {
    c4_pos->data[27].f1.size[0] = 4;
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel177(
  const int32_T c4_positionOut[112], c4_emxArray_cell_wrap_21_28 *c4_pos)
{
  int32_T c4_i126;
  c4_i126 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i126 < 4) {
    c4_pos->data[27].f1.data[c4_i126] = c4_positionOut[28 * c4_i126 + 27];
  }
}

static __global__ __launch_bounds__(32, 1) void c4_eML_blk_kernel_kernel178(
  const c4_emxArray_cell_wrap_21_28 *c4_pos, int32_T c4_positionOut_data[112])
{
  int32_T c4_out[4];
  int32_T c4_i129;
  int32_T c4_i130;
  int32_T c4_ii;
  c4_ii = (int32_T)mwGetGlobalThreadIndex();
  if (c4_ii < 28) {
    for (c4_i130 = 0; c4_i130 < 4; c4_i130++) {
      c4_out[c4_i130] = c4_pos->data[c4_ii].f1.data[c4_i130];
    }

    for (c4_i129 = 0; c4_i129 < 4; c4_i129++) {
      c4_positionOut_data[c4_ii + 28 * c4_i129] = c4_out[c4_i129];
    }
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel179
  (real32_T c4_c_In[921600], real32_T c4_b_I[921600])
{
  int32_T c4_i128;
  c4_i128 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i128 < 921600) {
    c4_b_I[c4_i128] = c4_c_In[c4_i128];
    c4_c_In[c4_i128] = 0.0F;
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel180
  (uint8_T c4_pixCount[640])
{
  int32_T c4_i131;
  c4_i131 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i131 < 640) {
    c4_pixCount[c4_i131] = (uint8_T)0U;
  }
}

static __global__ __launch_bounds__(128, 1) void c4_eML_blk_kernel_kernel181(
  const int32_T c4_positionOut_data[112], int32_T c4_b_positionOut_data[112])
{
  int32_T c4_i132;
  c4_i132 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i132 < 112) {
    c4_b_positionOut_data[c4_i132] = c4_positionOut_data[c4_i132];
  }
}

static __global__ __launch_bounds__(96, 1) void c4_eML_blk_kernel_kernel182(
  const int8_T c4_color[84], real32_T c4_b_color[84])
{
  int32_T c4_i133;
  c4_i133 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i133 < 84) {
    c4_b_color[c4_i133] = (real32_T)c4_color[c4_i133];
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel183(
  const real32_T c4_c_In[921600], real32_T c4_b_I[921600])
{
  int32_T c4_i134;
  c4_i134 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i134 < 921600) {
    c4_b_I[c4_i134] = c4_c_In[c4_i134];
  }
}

static __global__ __launch_bounds__(64, 1) void c4_eML_blk_kernel_kernel184(
  const real32_T c4_b_ltPts[56], int32_T c4_position[56])
{
  int32_T c4_i135;
  int32_T c4_i137;
  real32_T c4_f2;
  c4_i135 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i135 < 56) {
    c4_f2 = roundf(c4_b_ltPts[c4_i135]);
    if (c4_f2 < 2.14748365E+9F) {
      if (c4_f2 >= -2.14748365E+9F) {
        c4_i137 = (int32_T)c4_f2;
      } else {
        c4_i137 = MIN_int32_T;
      }
    } else if (c4_f2 >= 2.14748365E+9F) {
      c4_i137 = MAX_int32_T;
    } else {
      c4_i137 = 0;
    }

    c4_position[c4_i135] = c4_i137;
  }
}

static __global__ __launch_bounds__(96, 1) void c4_eML_blk_kernel_kernel185(
  const int8_T c4_fv1[84], real32_T c4_color[84])
{
  int32_T c4_i136;
  c4_i136 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i136 < 84) {
    c4_color[c4_i136] = (real32_T)c4_fv1[c4_i136];
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel186
  (uint8_T c4_pixCount[640])
{
  int32_T c4_i138;
  c4_i138 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i138 < 640) {
    c4_pixCount[c4_i138] = (uint8_T)0U;
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel187(
  const real32_T c4_c_In[921600], real32_T c4_b_I[921600])
{
  int32_T c4_i139;
  c4_i139 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i139 < 921600) {
    c4_b_I[c4_i139] = c4_c_In[c4_i139];
  }
}

static __global__ __launch_bounds__(64, 1) void c4_eML_blk_kernel_kernel188(
  const real32_T c4_b_rtPts[56], int32_T c4_position[56])
{
  int32_T c4_i140;
  int32_T c4_i142;
  real32_T c4_f3;
  c4_i140 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i140 < 56) {
    c4_f3 = roundf(c4_b_rtPts[c4_i140]);
    if (c4_f3 < 2.14748365E+9F) {
      if (c4_f3 >= -2.14748365E+9F) {
        c4_i142 = (int32_T)c4_f3;
      } else {
        c4_i142 = MIN_int32_T;
      }
    } else if (c4_f3 >= 2.14748365E+9F) {
      c4_i142 = MAX_int32_T;
    } else {
      c4_i142 = 0;
    }

    c4_position[c4_i140] = c4_i142;
  }
}

static __global__ __launch_bounds__(96, 1) void c4_eML_blk_kernel_kernel189(
  const int8_T c4_fv1[84], real32_T c4_color[84])
{
  int32_T c4_i141;
  c4_i141 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i141 < 84) {
    c4_color[c4_i141] = (real32_T)c4_fv1[c4_i141];
  }
}

static __global__ __launch_bounds__(512, 1) void c4_eML_blk_kernel_kernel190
  (uint8_T c4_pixCount[640])
{
  int32_T c4_i143;
  c4_i143 = (int32_T)mwGetGlobalThreadIndex();
  if (c4_i143 < 640) {
    c4_pixCount[c4_i143] = (uint8_T)0U;
  }
}

static void init_dsm_address_info(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
}

static void init_simulink_io_address(SFc4_LaneDetectionInstanceStruct
  *chartInstance)
{
  chartInstance->c4_fEmlrtCtx = (void *)sfrtGetEmlrtCtx(chartInstance->S);
  chartInstance->c4_laneFound = (boolean_T *)ssGetInputPortSignal_wrapper
    (chartInstance->S, 0);
  chartInstance->c4_ltPts = (real32_T (*)[56])ssGetInputPortSignal_wrapper
    (chartInstance->S, 1);
  chartInstance->c4_rtPts = (real32_T (*)[56])ssGetInputPortSignal_wrapper
    (chartInstance->S, 2);
  chartInstance->c4_bboxes_data = (real_T (*)[80])ssGetInputPortSignal_wrapper
    (chartInstance->S, 3);
  chartInstance->c4_bboxes_sizes = (int32_T (*)[2])
    ssGetCurrentInputPortDimensions_wrapper(chartInstance->S, 3);
  chartInstance->c4_scores_data = (real32_T (*)[20])ssGetInputPortSignal_wrapper
    (chartInstance->S, 4);
  chartInstance->c4_scores_sizes = (int32_T (*)[2])
    ssGetCurrentInputPortDimensions_wrapper(chartInstance->S, 4);
  chartInstance->c4_In = (real32_T (*)[921600])ssGetInputPortSignal_wrapper
    (chartInstance->S, 5);
  chartInstance->c4_b_In = (real32_T (*)[921600])ssGetOutputPortSignal_wrapper
    (chartInstance->S, 1);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* SFunction Glue Code */
void sf_c4_LaneDetection_get_check_sum(mxArray *plhs[])
{
  ((real_T *)mxGetPr((plhs[0])))[0] = (real_T)(2683009281U);
  ((real_T *)mxGetPr((plhs[0])))[1] = (real_T)(201091609U);
  ((real_T *)mxGetPr((plhs[0])))[2] = (real_T)(1428204615U);
  ((real_T *)mxGetPr((plhs[0])))[3] = (real_T)(2842182956U);
}

mxArray *sf_c4_LaneDetection_third_party_uses_info(void)
{
  mxArray * mxcell3p = mxCreateCellMatrix(1,2);
  mxSetCell(mxcell3p, 0, mxCreateString(
             "vision.internal.buildable.insertShapeBuildable"));
  mxSetCell(mxcell3p, 1, mxCreateString(
             "vision.internal.buildable.insertMarkerBuildable"));
  return(mxcell3p);
}

mxArray *sf_c4_LaneDetection_jit_fallback_info(void)
{
  const char *infoFields[] = { "fallbackType", "fallbackReason",
    "hiddenFallbackType", "hiddenFallbackReason", "incompatibleSymbol" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 5, infoFields);
  mxArray *fallbackType = mxCreateString("pre");
  mxArray *fallbackReason = mxCreateString("GPUAcceleration");
  mxArray *hiddenFallbackType = mxCreateString("late");
  mxArray *hiddenFallbackReason = mxCreateString("ir_functions");
  mxArray *incompatibleSymbol = mxCreateString(
    "insertShapeBuildable_drawBaseConstruct");
  mxSetField(mxInfo, 0, infoFields[0], fallbackType);
  mxSetField(mxInfo, 0, infoFields[1], fallbackReason);
  mxSetField(mxInfo, 0, infoFields[2], hiddenFallbackType);
  mxSetField(mxInfo, 0, infoFields[3], hiddenFallbackReason);
  mxSetField(mxInfo, 0, infoFields[4], incompatibleSymbol);
  return mxInfo;
}

mxArray *sf_c4_LaneDetection_updateBuildInfo_args_info(void)
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

static const mxArray *sf_get_sim_state_info_c4_LaneDetection(void)
{
  const char *infoFields[] = { "chartChecksum", "varInfo" };

  mxArray *mxInfo = mxCreateStructMatrix(1, 1, 2, infoFields);
  mxArray *mxVarInfo = sf_mex_decode(
    "eNpjYPT0ZQACPiD+wcjAwAakOYCYiQECWKF8RqgYI1ycBS6uAMQllQWpIPHiomTPFCCdl5gL5ie"
    "WVnjmpeWDzbdgQJjPhsV8RiTzOaHiEPDBnjL9Ig4g/QZI+lkI+E8AyPPMg4QLLHzIt1/BgTL9EP"
    "sDCLhfCsX9EH5mcXxicklmWWp8skm8T2JeqktqSSpQID8PYS4IAAB79hon"
    );
  mxArray *mxChecksum = mxCreateDoubleMatrix(1, 4, mxREAL);
  sf_c4_LaneDetection_get_check_sum(&mxChecksum);
  mxSetField(mxInfo, 0, infoFields[0], mxChecksum);
  mxSetField(mxInfo, 0, infoFields[1], mxVarInfo);
  return mxInfo;
}

static const char* sf_get_instance_specialization(void)
{
  return "so1Ri5ewkQbm0d2yD9EK30B";
}

static void sf_opaque_initialize_c4_LaneDetection(void *chartInstanceVar)
{
  initialize_params_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
    chartInstanceVar);
  initialize_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_enable_c4_LaneDetection(void *chartInstanceVar)
{
  enable_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_disable_c4_LaneDetection(void *chartInstanceVar)
{
  disable_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*) chartInstanceVar);
}

static void sf_opaque_gateway_c4_LaneDetection(void *chartInstanceVar)
{
  sf_gateway_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static const mxArray* sf_opaque_get_sim_state_c4_LaneDetection(SimStruct* S)
{
  return get_sim_state_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct *)
    sf_get_chart_instance_ptr(S));     /* raw sim ctx */
}

static void sf_opaque_set_sim_state_c4_LaneDetection(SimStruct* S, const mxArray
  *st)
{
  set_sim_state_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
    sf_get_chart_instance_ptr(S), st);
}

static void sf_opaque_cleanup_runtime_resources_c4_LaneDetection(void
  *chartInstanceVar)
{
  if (chartInstanceVar!=NULL) {
    SimStruct *S = ((SFc4_LaneDetectionInstanceStruct*) chartInstanceVar)->S;
    if (sim_mode_is_rtw_gen(S) || sim_mode_is_external(S)) {
      sf_clear_rtw_identifier(S);
      unload_LaneDetection_optimization_info();
    }

    mdl_cleanup_runtime_resources_c4_LaneDetection
      ((SFc4_LaneDetectionInstanceStruct*) chartInstanceVar);
    ((SFc4_LaneDetectionInstanceStruct*) chartInstanceVar)->
      ~SFc4_LaneDetectionInstanceStruct();
    utFree(chartInstanceVar);
    if (ssGetUserData(S)!= NULL) {
      sf_free_ChartRunTimeInfo(S);
    }

    ssSetUserData(S,NULL);
  }
}

static void sf_opaque_mdl_start_c4_LaneDetection(void *chartInstanceVar)
{
  mdl_start_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_mdl_terminate_c4_LaneDetection(void *chartInstanceVar)
{
  mdl_terminate_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

static void sf_opaque_init_subchart_simstructs(void *chartInstanceVar)
{
  initSimStructsc4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
    chartInstanceVar);
}

extern unsigned int sf_machine_global_initializer_called(void);
static void mdlProcessParameters_c4_LaneDetection(SimStruct *S)
{
  int i;
  for (i=0;i<ssGetNumRunTimeParams(S);i++) {
    if (ssGetSFcnParamTunable(S,i)) {
      ssUpdateDlgParamAsRunTimeParam(S,i);
    }
  }

  sf_warn_if_symbolic_dimension_param_changed(S);
  if (sf_machine_global_initializer_called()) {
    initialize_params_c4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
      sf_get_chart_instance_ptr(S));
    initSimStructsc4_LaneDetection((SFc4_LaneDetectionInstanceStruct*)
      sf_get_chart_instance_ptr(S));
  }
}

const char* sf_c4_LaneDetection_get_post_codegen_info(void)
{
  int i;
  const char* encStrCodegen [32] = {
    "eNrtXM1v40QUT6pSsRKsKoGEtEJixQVOpE1SBBfoNh8iot0tTXc5RjPjST3EHruecbJlT/v1d7A",
    "Slz1y4AD/CUeuICFx5MgbJ02TiZ1sWrbJi7CUprbfPP/mfXv8nFy+cZCD7SZ8fndyuQ34fhM+a7",
    "n+9sZgPz/y6R9fz3002P8ZBsnYPyQR8VVu6iaJz4+4CrxYi0A2ZDtIJROyzSMuGdCGQaSzuCnhx",
    "56QnXosmeGnvnUFc5tuEHvOHowlzj3pnQG3MNaHwKcqIs50nXNHu1EQn7h1j5wMEUe6V3E566jY",
    "nzYFxXUzDg0sdRB7WoQerz3krCGVJoBYXWBraqJ5RT/MnKaZqWqeEwZ+6AkiU2frEtXkIQhY8/u",
    "hA3/vxRomZZMxl0R6j7uky9W+6CQ8A8ltnkLBCSok0UEkiFfzvYoZOInt0AM8B4HDvSkCAWx7ES",
    "edMBBSZ+u/WYeZ1iShHq9yGp9kc2vy09go/4HgPR5lyq1dCbo8Iif8nsy8aCKQ2sNEW0MrmSTTw",
    "ucPSHSHgf4UdzKtFyxHNQnoiR/DiCwynkyyoY4j0QXxZnKL/YaxzFkuE/t9ZatZZAm3WpdP08KQ",
    "W53JCvE8lUl2HIT7vMu9hGuVaDKdrM81nU4p4RwHIGBj3tneEEsBih+QVQLpiFR1dS2CJO7chcA",
    "yTslipQO/AsZb3d+fPD1J1pCaR23CeFoUiIhQHGSWiDebmyOU0T0QAiqdwEsj7lvILKqcasey2g",
    "uiDshkShC5mILRaCahr05Al+AJ9xU4zTQyo8tZdIwwlzsmwAiPH4DbAG2KTJQJbXfA77pCn1W5Y",
    "pEIU7Qag9dBGKoZgzoL+X3ZkUFP1qPAbw5ifF8LEBkghvugg+PExyQDVkJpCBfi4vIO52CUJJJC",
    "nuxBmIvO6gAyVWMm723lLvLeW6+Q987H2d8fj/DJp/DJjXwb+s9G6G+sjdNvWNddOz+WbP3xuyP",
    "j37aut26NN3Sb8Cn90P3jnT8f//rh7u0XfxW+/9Gev40jP4Ejn/xvxr1cm69euDnYf/88MA8drT",
    "th34b2qxFc6yn83xvhvznYV8H2kdjhvc431N9yimfVz2tfl7b2En7/5KfjXbPwnh+/bTIEWGViz",
    "xFrOINCxuyTuJ/ebX1uzJDHjcHx/vb3l1cb/+6urcf1GfPbhL2GHLffy1//9u7VxvevfzgD/y1L",
    "37eSOqBFTHThLVZu7RMJtYXmSZqf9M/L2vm843LXPA4Lzv/HvX69v0r+2rjkuPwV8+V1jbvq/Ob",
    "N48tGPy0P5Cz6zSWex1Xrq9dN/1tuvvrng8H+F8NbooorPCelOh6chhq2nXZ2Rew0PJpOH1r0v8",
    "yoDx5bdm32C27g84LrM69wpwo3CzoKvILywui7Qku1zddY0QAHudcumBuvApR6BSZl6yzwgm7xi",
    "Ku7XO9sPeCuYGbJJ1kG2BqvObYGxHBv2i226CdUSLT4t1u9OfE/t/A/Xxx+jyhlJjGcA1b8dA78",
    "zyz8zxaAP+Kq3G7RiEjmFtmFCSHFTzHi58jlP8RP0ePHbT8Euf9itx+C034c5PHTQW7/DnL7Ycj",
    "zF0Nu/2xV7J+ilz9K/BS5/1Lk/kuR1z8Ut/8WKe74OYofpfzJqsjf8t8XM/AHFv7gv8GvIyIkdw",
    "wdzGACcZuZA9sDU8EkZ4Y8TpJL+ulTC//TBfvp9shK+bLbeRGfnZdIRj79aQb+Rxb+RwvAz6wnE",
    "Sjz0Jx18PL5J3L5Y78PpMjrsDnvo55Y+J8sAH/yzkBruxW024rriwehKOM+zjq+RFblOSLSdRyC",
    "fB2knRE/Z9WXvoXfv5b6ko31jGDy08uuFy9Dv8t5nFeMeNzqd1leOynPZSfL1hfVQyPn0lz9Q8v",
    "ij3RV8j7O9buSc8n1u8XZ+c6YnS9zfkToj0Xkz8NKzuqsp1M86+m94TrjMudHiiJu7KCsq+mq9M",
    "GgyYPbGO2kmNWvgGd9n2JcXy6xVVkfIVj8szxW/72cgfPUwnl6LTgdosnEsvErybVj4e1cU73x6",
    "ZzPXxeHs4c9TvSQv09CkfeDorx/Qd4PWkTeT1xaGflT5PgZevwY+0DKWX1aSPCXVgc/zvqH4M6/",
    "JeTPwUvI+9FLyN8HGH0fsoe6j4sir98ocv/FL3/sz78otv6W0d9zmff3aJbwfR6KvB8Ad/xh8z0",
    "/DS384fWtvw7buVDaCUNu5+jvc1HeZ5WR902VCcI+jeJInwZCOeN+P5zg6StA2LdWJqvyOw4jde",
    "O/EDyhLw==",
    ""
  };

  static char newstr [2261] = "";
  newstr[0] = '\0';
  for (i = 0; i < 32; i++) {
    strcat(newstr, encStrCodegen[i]);
  }

  return newstr;
}

static void mdlSetWorkWidths_c4_LaneDetection(SimStruct *S)
{
  const char* newstr = sf_c4_LaneDetection_get_post_codegen_info();
  sf_set_work_widths(S, newstr);
  ssSetChecksum0(S,(3916865331U));
  ssSetChecksum1(S,(3112299027U));
  ssSetChecksum2(S,(2686468130U));
  ssSetChecksum3(S,(2742693869U));
}

static void mdlRTW_c4_LaneDetection(SimStruct *S)
{
  if (sim_mode_is_rtw_gen(S)) {
    ssWriteRTWStrParam(S, "StateflowChartType", "Embedded MATLAB");
  }
}

static void mdlSetupRuntimeResources_c4_LaneDetection(SimStruct *S)
{
  SFc4_LaneDetectionInstanceStruct *chartInstance;
  chartInstance = (SFc4_LaneDetectionInstanceStruct *)utMalloc(sizeof
    (SFc4_LaneDetectionInstanceStruct));
  if (chartInstance==NULL) {
    sf_mex_error_message("Could not allocate memory for chart instance.");
  }

  memset(chartInstance, 0, sizeof(SFc4_LaneDetectionInstanceStruct));
  chartInstance = new (chartInstance) SFc4_LaneDetectionInstanceStruct;
  chartInstance->chartInfo.chartInstance = chartInstance;
  chartInstance->chartInfo.isEMLChart = 1;
  chartInstance->chartInfo.chartInitialized = 0;
  chartInstance->chartInfo.sFunctionGateway = sf_opaque_gateway_c4_LaneDetection;
  chartInstance->chartInfo.initializeChart =
    sf_opaque_initialize_c4_LaneDetection;
  chartInstance->chartInfo.mdlStart = sf_opaque_mdl_start_c4_LaneDetection;
  chartInstance->chartInfo.mdlTerminate =
    sf_opaque_mdl_terminate_c4_LaneDetection;
  chartInstance->chartInfo.mdlCleanupRuntimeResources =
    sf_opaque_cleanup_runtime_resources_c4_LaneDetection;
  chartInstance->chartInfo.enableChart = sf_opaque_enable_c4_LaneDetection;
  chartInstance->chartInfo.disableChart = sf_opaque_disable_c4_LaneDetection;
  chartInstance->chartInfo.getSimState =
    sf_opaque_get_sim_state_c4_LaneDetection;
  chartInstance->chartInfo.setSimState =
    sf_opaque_set_sim_state_c4_LaneDetection;
  chartInstance->chartInfo.getSimStateInfo =
    sf_get_sim_state_info_c4_LaneDetection;
  chartInstance->chartInfo.zeroCrossings = NULL;
  chartInstance->chartInfo.outputs = NULL;
  chartInstance->chartInfo.derivatives = NULL;
  chartInstance->chartInfo.mdlRTW = mdlRTW_c4_LaneDetection;
  chartInstance->chartInfo.mdlSetWorkWidths = mdlSetWorkWidths_c4_LaneDetection;
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
    chartInstance->c4_JITStateAnimation,
    chartInstance->c4_JITTransitionAnimation);
  init_dsm_address_info(chartInstance);
  init_simulink_io_address(chartInstance);
  if (!sim_mode_is_rtw_gen(S)) {
  }

  mdl_setup_runtime_resources_c4_LaneDetection(chartInstance);
}

void c4_LaneDetection_method_dispatcher(SimStruct *S, int_T method, void *data)
{
  switch (method) {
   case SS_CALL_MDL_SETUP_RUNTIME_RESOURCES:
    mdlSetupRuntimeResources_c4_LaneDetection(S);
    break;

   case SS_CALL_MDL_SET_WORK_WIDTHS:
    mdlSetWorkWidths_c4_LaneDetection(S);
    break;

   case SS_CALL_MDL_PROCESS_PARAMETERS:
    mdlProcessParameters_c4_LaneDetection(S);
    break;

   default:
    /* Unhandled method */
    sf_mex_error_message("Stateflow Internal Error:\n"
                         "Error calling c4_LaneDetection_method_dispatcher.\n"
                         "Can't handle method %d.\n", method);
    break;
  }
}
