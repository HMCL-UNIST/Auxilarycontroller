#ifndef __c3_LaneDetection_h__
#define __c3_LaneDetection_h__

/* Type Definitions */
#ifndef struct_tag_dK05fgDE5TQ2s3o7OZp84G
#define struct_tag_dK05fgDE5TQ2s3o7OZp84G

struct tag_dK05fgDE5TQ2s3o7OZp84G
{
  char_T f1[6];
  char_T f2[6];
};

#endif                                 /*struct_tag_dK05fgDE5TQ2s3o7OZp84G*/

#ifndef typedef_c3_s_dK05fgDE5TQ2s3o7OZp84G
#define typedef_c3_s_dK05fgDE5TQ2s3o7OZp84G

typedef tag_dK05fgDE5TQ2s3o7OZp84G c3_s_dK05fgDE5TQ2s3o7OZp84G;

#endif                                 /*typedef_c3_s_dK05fgDE5TQ2s3o7OZp84G*/

#ifndef struct_tag_cusgLjlvdWnAmMLbzAUfJ
#define struct_tag_cusgLjlvdWnAmMLbzAUfJ

struct tag_cusgLjlvdWnAmMLbzAUfJ
{
  char_T f1[4];
  real_T f2[2];
  char_T f3[6];
  char_T f4[6];
};

#endif                                 /*struct_tag_cusgLjlvdWnAmMLbzAUfJ*/

#ifndef typedef_c3_cell_1
#define typedef_c3_cell_1

typedef tag_cusgLjlvdWnAmMLbzAUfJ c3_cell_1;

#endif                                 /*typedef_c3_cell_1*/

#ifndef struct_tag_SNeuYARkwpYnGEDfxo074B
#define struct_tag_SNeuYARkwpYnGEDfxo074B

struct tag_SNeuYARkwpYnGEDfxo074B
{
  char_T f1[2];
  char_T f2[9];
};

#endif                                 /*struct_tag_SNeuYARkwpYnGEDfxo074B*/

#ifndef typedef_c3_cell_2
#define typedef_c3_cell_2

typedef tag_SNeuYARkwpYnGEDfxo074B c3_cell_2;

#endif                                 /*typedef_c3_cell_2*/

#ifndef struct_tag_2Otzot4aR0pW68wjnUjAj
#define struct_tag_2Otzot4aR0pW68wjnUjAj

struct tag_2Otzot4aR0pW68wjnUjAj
{
  c3_s_dK05fgDE5TQ2s3o7OZp84G _data;
};

#endif                                 /*struct_tag_2Otzot4aR0pW68wjnUjAj*/

#ifndef typedef_c3_s_2Otzot4aR0pW68wjnUjAj
#define typedef_c3_s_2Otzot4aR0pW68wjnUjAj

typedef tag_2Otzot4aR0pW68wjnUjAj c3_s_2Otzot4aR0pW68wjnUjAj;

#endif                                 /*typedef_c3_s_2Otzot4aR0pW68wjnUjAj*/

#ifndef struct_tag_jYR3k7AHcxQEDXkuiJtXiG
#define struct_tag_jYR3k7AHcxQEDXkuiJtXiG

struct tag_jYR3k7AHcxQEDXkuiJtXiG
{
  c3_cell_1 _data;
};

#endif                                 /*struct_tag_jYR3k7AHcxQEDXkuiJtXiG*/

#ifndef typedef_c3_s_jYR3k7AHcxQEDXkuiJtXiG
#define typedef_c3_s_jYR3k7AHcxQEDXkuiJtXiG

typedef tag_jYR3k7AHcxQEDXkuiJtXiG c3_s_jYR3k7AHcxQEDXkuiJtXiG;

#endif                                 /*typedef_c3_s_jYR3k7AHcxQEDXkuiJtXiG*/

#ifndef struct_tag_zKb1MDLomfLiY5CbhTw1SG
#define struct_tag_zKb1MDLomfLiY5CbhTw1SG

struct tag_zKb1MDLomfLiY5CbhTw1SG
{
  c3_cell_2 _data;
};

#endif                                 /*struct_tag_zKb1MDLomfLiY5CbhTw1SG*/

#ifndef typedef_c3_s_zKb1MDLomfLiY5CbhTw1SG
#define typedef_c3_s_zKb1MDLomfLiY5CbhTw1SG

typedef tag_zKb1MDLomfLiY5CbhTw1SG c3_s_zKb1MDLomfLiY5CbhTw1SG;

#endif                                 /*typedef_c3_s_zKb1MDLomfLiY5CbhTw1SG*/

#ifndef struct_SFc3_LaneDetectionInstanceStruct
#define struct_SFc3_LaneDetectionInstanceStruct

struct SFc3_LaneDetectionInstanceStruct
{
  SimStruct *S;
  ChartInfoStruct chartInfo;
  uint8_T c3_is_active_c3_LaneDetection;
  uint8_T c3_JITStateAnimation[1];
  uint8_T c3_JITTransitionAnimation[1];
  real_T c3_laneCoeffMeans[6];
  boolean_T c3_laneCoeffMeans_not_empty;
  real_T c3_laneCoeffStds[6];
  boolean_T c3_laneCoeffStds_not_empty;
  real_T c3_vehicleXPoints[28];
  boolean_T c3_vehicleXPoints_not_empty;
  void *c3_fEmlrtCtx;
  real32_T (*c3_laneNetOut)[6];
  boolean_T *c3_laneFound;
  real32_T (*c3_ltPts)[56];
  real32_T (*c3_rtPts)[56];
  real32_T (*c3_gpu_U)[56];
  real_T *c3_gpu_t2;
  real32_T (*c3_gpu_ltPts)[56];
  real_T (*c3_gpu_Tinv)[9];
  int32_T *c3_gpu_p3;
  real_T (*c3_b_gpu_Tinv)[9];
  real_T (*c3_gpu_dv)[6];
  real32_T (*c3_b_gpu_U)[84];
  real32_T (*c3_gpu_b)[56];
  real32_T (*c3_gpu_fv)[84];
  real32_T (*c3_gpu_rt_y)[28];
  real32_T *c3_gpu_params;
  real32_T (*c3_b_gpu_params)[6];
  real_T *c3_gpu_t3;
  int32_T *c3_gpu_p2;
  real_T (*c3_gpu_T)[9];
  real32_T (*c3_gpu_laneNetOut)[6];
  int32_T *c3_gpu_p1;
  real_T (*c3_gpu_x)[9];
  real_T *c3_b_gpu_t3;
  real_T (*c3_gpu_dv1)[6];
  real32_T (*c3_c_gpu_U)[56];
  int32_T *c3_b_gpu_p3;
  real32_T (*c3_gpu_fv1)[84];
  real32_T (*c3_gpu_lt_y)[28];
  real_T *c3_b_gpu_t2;
  real32_T (*c3_gpu_rtPts)[56];
  real32_T *c3_c_gpu_params;
};

#endif                                 /*struct_SFc3_LaneDetectionInstanceStruct*/

/* Named Constants */

/* Variable Declarations */

/* Variable Definitions */

/* Function Declarations */
extern const mxArray *sf_c3_LaneDetection_get_eml_resolved_functions_info();

/* Function Definitions */
extern void sf_c3_LaneDetection_get_check_sum(mxArray *plhs[]);
extern void c3_LaneDetection_method_dispatcher(SimStruct *S, int_T method, void *
  data);

#endif
