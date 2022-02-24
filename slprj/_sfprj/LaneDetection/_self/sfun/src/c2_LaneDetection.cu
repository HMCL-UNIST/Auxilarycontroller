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
    "789ced9dd98feb585ec7d3a31ed10d736f173d0cd38c586e0b9ea6e156924a556e7881c449aa92cabe5496ab5197e39c244e1cdbf192ed85f0366f20010f68d8"
    "d10c8c18864602346c7f03affc1d3cf18284b39caac437be4e979d731dfb67a93b6dfd5ce7fbfb9d73fa7c7c56fbde4b65dff3f97ccfb57ffef8777cbee9f3af",
    "f896d733dffa3adbfc7ec5b77be9edef6d7eabba7b7c7dd5f7fecedf2ded7fa8e9fddee69e1178054d95f50d4f0fd1c35fb68521cbd3bc529989c8272159e0c6"
    "a8bdb274580e55d8212a6fdfe49677c3e496e9e166695afe37d543cca0ac0e7d524f7ef490dbbe59e5c7f2fa6f8378df3f303f4606f971a6b3bf4ed513df3957",
    "65e99c13189a3bcf462b9968ecbc14f407fdad734510b896303d977bb484dae76d8e11da48fabc45cb68f7e6b3d57f9f7fc66ab929f15a3a9c40b7e308891944"
    "4b3ccb7773489908d2e0a5a88bf3bb16e3fcd4244e6c1fd20a47b728cdcb2ee2735aa1ab3cdde25041124424292cc26580fd5a58f4eb174cfcc2f655bebddc93",
    "533a7f3e78cf9a3f3f30f107db5fa7a8ef9cf784213aef0d19ee3c2e30ea10f18a8c2b4659154541520a3433a0bb483ea4a2c8eb3f11f19fac9ff9bccd7daed0"
    "521769696f6acf6fefc98673834a84f3e5de20ee8f0ecc17fdefe3f31fac7e7ff38bfff97cf94b4a0f5f5ed19b1aa47768bdfe7903bd339d7d30e98e2a2a956f",
    "86a36cb17417a877e862fffad18f82898e991f3e837b52e97b8517f8fea9713ed3ddebe3c476ad5ddab8906ff57dc005e0027e1eb840420fb8604ffac085c3e2"
    "fc9a499cd8aeb54b9555eb94615bdbfa0b8bfac005e0c253f4f0e5153de0823de97b850b56ebcbcf9ac489ed0f5c88d30abd1a32dcf56361d10fe003f0e1297a",
    "f8f28a1ef0c19ef4bdc207abfd869f368913dbb57629955fce36c8bbfa0b8bfac005e0c253f4f0e5153de0823de97b850b56ebcb374de2c47666b92620294865"
    "345211cf20dc2a031f800febe7810f24f4800ff6a4ef153e58ed3718e5eb99eef7b1dfa041a294cb0117800bebe7810b24f4800bf6a40f5c382cce67ba7b7d9c",
    "d83ed1dc48f1a2aa44a5aeec83f124e0027e1eb840420fb8604ffa5ee1c2bdc5383fd0dd3fc6b9b62c5724d1332401078003ebe7810324f48003f6a4ef150e58"
    "ed1f7c89f9e682d668e5d4e1aefec2a23e7001b8f0143d7c79450fb8604ffac085c3e2fc199338b17d22d162965ead52ddd15f58d4072e00179ea2872fafe801",
    "17ec49df2b5c58588cf35b267162fb98e6d836ade0f5474941a2385a96d9cecc5e7f8013c089a7e8e1cb2b7ac0097bd2f70a27acf61f0e9d77d6daa5151872f4"
    "10c1bc3370e1f179e002093de0823de97b850b0b8b711eda7fd8395f2f234c905411ca8aa4328abdfe002780134fd1c39757f48013f6a40f9c382cce43db6551",
    "42a216c36ae96a999da3e5b6060a8d69ee1df9039c004e6cebe1cb2b7ac0097bd2f70a27ac8e333dd7ddebe3c476f9e1fcd5dcea5076e0027061fd3c7081841e"
    "70c19ef4bdc2857b8b717ed530ceb545e3812afa8003c001fc3c7080841e70c09ef4bdc201abfd830f4de2c4762d36aebc61c2b6fec2a23e7001b8f0143d7c79",
    "450fb8604ffac085c3e23c747fc3860bcba9851dfd85457de00270e1297af8f28a1e70c19ef4810b87c579e87cc2920b25246b64506805e613800b0fcf031748"
    "e80117ec491fb870589c1fe9eef57162fb920b5571b9f36d0d06e0027061fd3c7081841e70c19ef4810b87c579e8773e1589e6655190d11d2d6deb2f2cea0317",
    "800b4fd1c39757f4800bf6a40f5c382c4ea37c3dd3fd8a481aaaca7affc2f21025e0027061fd3c7081841e70c19ef4bdc20552df7fde7021af2a1b30e8fc5858"
    "f403f8007c788a1ebebca2077cb0277de0c361717edd244e6c376a91810fc087f5f3c007127ac0077bd2f70a1fac8e2b1dfabd06696b6dd2b6fec2a2bedd5cf0",
    "59e4c29f99f883edaf53798781e1b18448f2e0b3fffb147870443d523cb899371a8352add9baab0493749fe9cbad5222063c38351e58ad2fdf308913db45cd7f"
    "9651a27c7b7b759273b97066910b3f34f107db9dc785bd2575f0fe78bbdab56f03278eaa478a13179162b03d977ba58b6433c167661d3552c9dd00274e8d13f7",
    "16e334fbde27b3f93a83537960b59ff02726fe60bbf378804b665d2548b557bf0eedff51f548b5ff09617a554cde95fc287cddcfb5ae6e02f3bb38f4134eaefd"
    "b73a6e74e87e679a51d831adb0022fefe82f2cea3b8d077f6ee20fb63b8f075b25a4d512183772871e291e8cab8956393c11a874466d33296ea6348603984738",
    "391edc5b8cf3a70ce35c5b36a310ae6dffbf67e20fb63baffddf94cca64640fbef0e3d52edffa87ccd15a609aa5d2f75a522f26785496a9c84f6ffd4daff85c5"
    "380ffd0ecf723f7361dde22cbfaca0ca8a30ccd03324c9f6fae3b4f9831f9bf883edcee3837189919c67fed1bfff2ebdfcf54afb4d5a8f142fa49b6c5189b523",
    "a1b490998da297dd71aa9381fec2c9f182d4be85add109adf529e5723a3f1616fd70dabad4ef9bf883edcee3c41b25b5aa3330bfe00e3d527ca0d20cea722c57"
    "8c31f5f8a010af7632dd09cc2f001f74cfede703057cd8fc3a9d0f14613ec078d371f548f121179ac9c5b252ef6bb5ba321cdf4cb24121047c38393ee0fba7c6",
    "f94c77af8f13dbc587918b75dfc1ad5cf84b137fb0dd795cd82921e2eb52810bc7d523b51ebd57caab955b3655e0a55059e2a3178af26a7b3d3a70617ffa4ee3"
    "02a9fab21cd58eee195b722a1fbcbb7f616f4901275ca647aaff30cb861896bfab2b0217bb18e7c7ddf8a41e8b0327bcc68943c79776673f61fec1b9e34b6f94",
    "148c2fb9488fd8f812dd134b73f63acd4ea9e93c298bd74a56a0800fa7c687ef5a8cf3d74ce2c4765190958224304896d767eb558412525489a76805750589d5"
    "82dff66b61d12fbb79f189455efcc4c41f6c771e2f0e29b99722a9f60ece553aae1e297ee46fee72d9215d8d4532f15cbf25de06c7374202f8716afcb03a3ff1",
    "5c77af8f13db7747379ccb09abfd8abf32f107db9dc789dd12c21505d635b9438fd478f3558dcd47b9fc4575d011e60d556c340a52ca075cf01a17becc77a237"
    "a3193bfa0b8bfa4edb3777bafba6b74a08f64dbb468f543fa15a28d5f957c97c65926ff673f2ab405251601f84f778f0a1499cd82e2325ca333d41d2e92f2cea",
    "3b8d077f6ae20fb63b8f070f2504fb1e5ca4476cdf03937a356b73d56e1ab506a31b55562bb32ceca3f61c0f0efd2eb4d6da6c5c28b373243b960756c78dfeda"
    "c41f6c77240fb64b88f0f91ac085e3ea91e24298ebcafe0b39a9e4b235a67115a7653e2ec27aa593e3c2c2629c2f4ce2c4769196e4f5d73f654a73b58bf8adb5",
    "93a55ccea99cb03a0ffd4f26fe60bbf338615662704eab9bf44871a3529c2bad2c1d1d5389cb616c30658ad93c82f125e086ee396c7f6b2b4401374e8c1b1469"
    "6ec0bcc471f54871c3df51a5cb89d8ebdee452b154313dbc8d481cac7ff51c377ed1244e6c7fb315dacc896e8ee2702a37aceea7fbc2c41f6c3f056e3c9618b9",
    "f5aec08be3ea91e2c555b6527cd54866a633a678d91ed27d7a52e5e03c0ee085ee39535e948017cbeb84785122c90b18973aae1e295edc46b9ebf22476a55cfa"
    "3ba98694e56a6abc0dfd8b93e385d5faf2b1499cd82e2365d5f62ce750b737603b951356e7bb7f60e20fb63b8f136f9614c9f3c1e1fba2c7d523c58720ad54c2",
    "d1d0fc2a47858581bfc8cc46391afa1327c70762fbafd73b78b75b9e7dc73a3996171ede7f7d40c941ffc22d7aa4f8a1a695fcf46e9251afb3a1122af5fcfe34"
    "3786f55227c78f85c5385f98c489ed7b5aa19d4384dcca8d139ef7362931582fe5263d52fbb363cd344b2955ea8aa61374ae39f7875348f501374e8d1bf8fea9",
    "717ecd244e6c5f8d8a6f35cd4ee584cf2227fec2c41f6c772027b64a08e6b5dda247aa1f2176c6b569b412192646219e91238940621485fd7827c70352ef0f5a"
    "73b4ed0e25f01db6eb732e17bc7b9ef8de9282f3c45da6476cfd93188917b2b73d75c8aae15436df0f29b9ce0d70e2d438b1b018a7ddedf3bd457fded7dd3ffa",
    "b3b6305a4e6eeb919bb776180e0c2a07a9760bce7d3dae1e290e0c26dd5145a5f2cd70942d96ee02f50e5decbb68bfddbdc1df9be5a3fe32ca47afe9fdfe13f5"
    "70fadf36d1c3f6d7d5ede6361a67045e9104ee5c91689647ed0ccd2fc7485e0e6965d7bf7b037dbbfe3ffda30fbfd122a9f793fffad6c724f5f075aaeddea1fd",
    "de49b4556ff46297d579dccf9553cacda53ce27cee69f75e587c2ffa0f83f4cf74f6d7a9c409bc177dca96509795b577eb381211df463cc322793bbf1606f971"
    "6aefd1faebd0761ddea3e13dda8d7af01e6d4ffac0932fc713abf5ee1393fcc2f636b709e225a3b6797efdefa8c8be2b8efcef13f570fa7f60a287ed0438d215",
    "d5b575bba63ce4b7566996597d8e339cf03979bf05e3f047d523c50d140ec462b9311319678afd4081123bbd70db45eb3e7d16b9f13d83f4cf7476e7cdd7890f"
    "67ec6fe7c7c2205ee867bc5d0ffa196fcf17e867bc5b3de867d8933ef0629717a4fa117b3caa2a2c27dbc5ad5f36f103db0db9b5f2e65d9c6be1ac6ab2ca867d",
    "305919c8ad17fd0de87f1c558fd83a20265dae33b5a81f5ddff9e772b0d00b556817ad17059eecf2443488f7d0fc78a6bbd7e707b66bee557976a4a21c3d443e"
    "fbfa3f7673c4b3e75eac41b2534c24fb23b0ffecb87aa4f81118cde3b369b5510976508049c53aad8b48cd45e75e003f76f941ee7c0c246adede214611a4bcaa",
    "94e9a1c8e9ce5ad8f66b61d12fbbb96275beec3f4dfcc176c7d59b35570e293ea2df9b83feca71f548addb6a4a4c33ac3651a7558c2a5c4395722ab7fd7e09bc"
    "d99fbe5779f3ab26f981ed9b062b35d43c7f0b6e5ccb9bc3d76738acdeecf0e66dc547f67c0de0cd71f548f12698e383831637ea46a84aaf920fa7b9eba2e003",
    "deb8953756ebd5cf99e407b66f1aac522e47218ed39a2c5155b6fd5858f4c369f32d7f6fe20fb63bae9eecf065a7b8e07bd92ed223355e5656e8e184a932d504"
    "ba8a24e424952d317558efe55a9e2c0ce23d343f7ec9243fb01d3750c2244bf757e7c9adc75e56c7cdd9e88fdd5cb17a1ee0bf98f883ed8eab2fbb5cd95b6c24",
    "e763a0bf725c3d527ca9fb67379de02bea82be4aa58bc1c1ed2853aac17c3ef0c5203f0efefed11b0dd56aac05e3c5b57cf967137fb0dd71f5c5882f8fc506f3"
    "2f2ed223c59748afc0d3b74995a302178559ab9bbd4cd7f804f005f8b23f3f5e98e407b66f1a2a4ae0d421bfe75dd8a97cb13aeff2af26fe60bbe3eacb0e5f0c",
    "8b8df8f984c099e3ea91e20c93efd4e74288495cbc1a8c86c560a0d6ba40d08f01ce18e4c7af98e407b6ef6db0b6ba324ee58c67bfb7f416ce6c7765802feed0"
    "2335af7fa33499543cc25f28627c1e4c8e6be34c7a7b9c1cf8b23ffd53e50bb1f3f7d70d5596e5d916ad303dfc253fa772c5eabcfe3f98f883ed8eab273b5cd1",
    "159756778027eed023f5ffbd22a692c130c3461ad2b8acc682edb930635c749e3af064972756d7257f6a921fd8be69a0d62b8e8c3e12eb732e5fac8e8ffd9b89"
    "3fd8eeb87ab3c317e3e2c30364c01b77e891e24de6b6af062e0b69aecef58b95bb40841f2b4de08d6b79b33088f748ebc81e1aac9d4f94dae88fd3c6c7dcb18e",
    "6c7fb1c13a32f7e8911a1f1bce2e2b54237fe5cfb5e6b98e9264269742d2077c712b5facd6abaf9be407b66bee51aaac08c30c3d435a332588cede5769757cec"
    "4726fe60bbe3eac9c3f930fae2827d946ed223c593347d35b84554207c9354d2f54248ca846e611fa57b79221ac46bf739634c0f3183542729706dd4f6399723",
    "1e3f676ca798a03fe21e3d52e35d0d3edcecb686acd264e5496630bda495801ff64dba961f04bf7fbe1e3429b3f3e58121abf75c9f733962b53ff263137fb0dd"
    "71f5e4a13ff26671014fdca3478a27f9f0ec3a8a2874d9ae246eb3c1503177a12660fe047862901f1f9be407b66beea5f2ab71922d98f8dccb93bf33f107db1d",
    "574f1e78a22b2ec2dff1029e1c578f144f7aa9693ecaf7fda3c43c9f0e2961999bcc86b05f057862901fdf34c90f6cd7dc2b6bbea3762abf6a9e52bce60c925d"
    "cb932f4cfcc176c7d593079eec2b2ed89fe2163d523c916f036c57be494f03a142b670a14c8bb3de18ced5772d4f48cd9768eee186094d7dceed97787cbe64a7",
    "98607ccb3d7aa4c6b54795423d58a2d237bd54a23bccb1135e2c367cc00fe0c7fefc30caff33ddafe6de72b35c6cb9592ec57704a7f2c36a3fe48726fe60bbe3"
    "eac7033f768a09ce6171991eb1efd3a707574d558d06c6c1162bcfb3c5f6655282712dd772646110afddfb4c96032568a4229e4119c47795dee6687538af787d",
    "39aebe3c8e6f19161bf453dca3478a2fb777fd5a253e8c0de779a424a4aa44453a330af8e256be58eda77ca4bbd7e707b63f2c14daec5c742a4fac8e73fdad89"
    "3fd8eeb8faa15bcf85772a023fdca1476a9c6b4e2b5ca5264523d7ea68242becabf2848af9801f6ee507f175c149442baa8496eb4d299f73fb25b02ef8cde282",
    "fe887bf448f547a886ec97ee62d52b6ada6af4aa5ce93a2066615db06b7942ea7bf6fb1aa83dc7ab38962f2f2cd69b13ff9efd21c507df5371911e29de94e249"
    "7f73fcaa381ff685eb191a6472513101df53712d6f08ae1bded928b769a91ccb175837bcafb8603ccc2d7aa4c62d4ae5666816ab4c1bd430cacd8a816eb8dddb",
    "3e170378b23f7daff2e4d0efd5af0ff0e01f5b2887f7573cfebdfa7dc505dfab77911e299e342337e3c6ddb89bacb25c6ddcbce5a56caae4039eb89527a241bc"
    "87e6c773ddbd3e3fb07db781722e47accecfff8d893fd8eeb8fab18f23f05d14d7e8911adf9adc06fad566bc12a6e7d94e2d5fcb052e151aceed722d3f48bd97",
    "8c698e6dd30ac2ab519dbe6ed8e3f3f37b8b0be6e7dda3478a27e5017d379f0af169ad36ed5ee4b2897848ad004f5ccb13abfd11a3fc3fd3fde2066ab5eb3a2a"
    "75ddca9113dfd7a82f26e2fb1a615cebb87aa438721798777b683abca85dc703bc1abb0dcbf5fa3570c4ad1cb15aaf3e31c90f6c5f0d9844f9764d732a294837",
    "34d759bdeb3a952756f733fea3893fd8eeb87ab235beb5a7b860bfbccbf4488d4bb4e6c2acd093b946fba62ede368b17b95211e6dfddcb958541bcefaa3dbf7f"
    "a23ffacbc81f7c915fb7e5aceab0c6879101d66db9458f547fe48a4997eb4c2dea47d777feb91c2cf44215da05e7acfc3f160e7e55",
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
  const char* encStrCodegen [31] = {
    "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",
    ""
  };

  static char newstr [2241] = "";
  newstr[0] = '\0';
  for (i = 0; i < 31; i++) {
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
