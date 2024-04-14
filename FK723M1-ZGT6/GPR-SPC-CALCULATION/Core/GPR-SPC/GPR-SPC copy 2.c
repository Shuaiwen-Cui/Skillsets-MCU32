/**
 * @file GPR-SPC.c
 * @author SHUAIWEN CUI (cswoffice@163.com)
 * @brief This file is the implementation of the GaussianProcessRegression class.
 * ! THEORETICAL BACKGROUND
 * ! [1] Pred = Kpt * Inv(Ktt + sigma^2 * I) * (y_train - mean(y_train)) + mean(y_train)
 * ! [2] Var = Kpp - Kpt * Inv(Ktt + sigma^2 * I) * Ktp
 * ! [3] Std = sqrt(Var)
 * ! [4] Stochastic Process Control: |GT_val - Pred_val| <= 2 * Std_val (95% confidence)
 * Where:
 * Kpt = K(x_pred, x_train)
 * Ktt = K(x_train, x_train)
 * Kpp = K(x_pred, x_pred)
 * Ktp = K(x_train, x_pred)
 * K is the kernel function, here we use RBF kernel:
 * ! K(p1, p2) = alpha^2 * exp(-(1/2) * sum((||p1[i] - p2[i]||/l[i]/l[i])^2)
 * where alpha is the signal variance, l is the length scale.
 * y_train is the training output, x_train is the training input, x_pred is the prediction input.
 * 
 * ! DIM_GPR_IN: the dimension of the input data, this can be multi dimensional
 * ! DIM_GPR_OUT: the dimension of the output data, this so far is single dimensional, the extension to multi dimensional is possible, but quite complex, not implemented here
 * ! In short, in this file, DIM_GPR_IN is multi-dimensional, DIM_GPR_OUT is single dimensional.
 * 
 * @version 0.9
 * @date 2024-04-14
 *
 * TODO LIST:
 * - 1. Verification
 * - 2. Expand to multi-dimensional output
 * - 3. Optimize the hyperparameters, so far it is hard coded, values from python script.
 */

/* SDCARD FILES ----------------------------------------------- START*/
/**
 * ! FILE STRUCTURE
 * - x_train.csv: the training input data 215 by 2
 * - y_train.csv: the training output data 215 by 1
 * - x_test.csv: the prediction input data 147 by 2
 * - y_testtrue.csv: the true output data for prediction 147 by 1
 * - y_testpred.csv: the predicted output data for prediction 147 by 1, to be filled by the algorithm
 * - y_testpred_var.csv: the variance data for prediction 147 by 1, to be filled by the algorithm
 * - y_testpred_std.csv: the standard deviation data for prediction 147 by 1, to be filled by the algorithm
 */

/* SDCARD FILES ------------------------------------------------- END*/

/* DEPENDENCIES ----------------------------------------------- START*/
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdlib.h> // for malloc
#include "led.h"
#include "usart.h"
#include "sdmmc_sd.h"
#include "ff.h"
#include "ff_gen_drv.h"
#include "sd_diskio.h"

#include <math.h>
#include "arm_math.h"

/* DEPENDENCIES ------------------------------------------------- END*/

/* STATEMENTS ------------------------------------------------- START*/
/* STATEMENTS ------------------------------------------------- START*/
/**
 * ! MACROS
 * @param ReadBufferSize The size of the buffer for reading data from the SD card.
 * @param DIM_GPR_IN The dimension of the input data for Gaussian Process Regression.
 * @param DIM_GPR_OUT The dimension of the output data for Gaussian Process Regression.
 * @param NUM_GPR_TRAIN The number of training data for Gaussian Process Regression.
 * @param NUM_GPR_PRED The number of prediction data for Gaussian Process Regression.
 */
#define ReadBufferSize 500
#define WriteBufferSize 500
#define DIM_GPR_IN 2
#define DIM_GPR_OUT 1
#define NUM_GPR_TRAIN 40
#define NUM_GPR_PRED 5

/**
 * ! FILE OPERATIONS
 * @param ret The result code of the file operation.
 * @param file The file object for the SD card operation.
 * @param ReadBuffer The buffer for reading data from the SD card.
 * @param WriteBuffer The buffer for writing data to the SD card.
 * @param token [pointer] The token for parsing the data.
 */
FRESULT ret; // Result code
FIL file;    // File object
UINT bytes_written;
char ReadBuffer[ReadBufferSize];
char WriteBuffer[WriteBufferSize];
char *token;

/**
 * ! GPR RELATED CALCULATION
 * @param IN_TRAIN The input data for training the GPR model.
 * @param OUT_TRAIN The output data for training the GPR model.
 * //@param OUT_TRAIN_MEAN The mean of the output data for training.
 * @param IN_PRED The input data for prediction.
 * @param OUT_GT The ground truth output data for prediction.
 * @param OUT_PRED The predicted output data.
 * //@param IN_TRAIN_AVG The average of the input data for training.
 * @param OUT_TRAIN_AVG The average of the output data for training.
 * //@param IN_PRED_AVG The average of the input data for prediction.
 * @param OUT_GT_AVG The average of the ground truth output data for prediction.
 * @param OUT_PRED_VAR The variance of the predicted output data.
 * @param OUT_PRED_STD The standard deviation of the predicted output data.
 *
 * //@param pKtt [pointer] The kernel matrix Ktt, for computing, it is in the form of a 1D array.
 * @param pKttn [pointer] The noised kernel matrix Kttn, for computing, it is in the form of a 1D array.
 * @param pKpt [pointer] The kernel matrix Kpt, for computing, it is in the form of a 1D array.
 * @param pKtp [pointer] The kernel matrix Ktp, for computing, it is in the form of a 1D array.
 * @param pKpp [pointer] The kernel matrix Kpp, for computing, it is in the form of a 1D array.
 * @param pIKttn [pointer] The inverse of the noised kernel matrix Kttn, for computing, it is in the form of a 1D array.
 * @param pKptxIKttn [pointer] The product of Kpt and IKttn, for computing, it is in the form of a 1D array.
 * // @param pIn_Train_Mean [pointer] The mean of the input data for training, for computing, it is in the form of a 1D array.
 * @param pOut_Train_Mean [pointer] The mean of the output data for training, for computing, it is in the form of a 1D array.
 * // @param pIn_Pred_Mean [pointer] The mean of the input data for prediction, for computing, it is in the form of a 1D array.
 * @param pOut_GT_Mean [pointer] The mean of the ground truth output data for prediction, for computing, it is in the form of a 1D array.
 * @param pDiff_Out_Train_Mean [pointer] The difference between OUT_TRAIN and OUT_TRAIN_AVG, for computing, it is in the form of a 1D array.
 * @param pOut_Pred_T1 [pointer] The predicted output Term 1, Kpt * Inv(Ktt + sigma^2 * I) * (y_train - mean(y_train)), for computing, it is in the form of a 1D array.
 * @param pOut_Pred_Mean [pointer] The mean of the predicted output data, for computing, it is in the form of a 1D array.
 * @param pOut_Pred [pointer] The predicted output data, for computing, it is in the form of a 1D array.
 * @param pOut_Pred_Var_T2 [pointer] The variance of the predicted output data, Kpt * Inv(Ktt + sigma^2 * I) * Ktp, careful with the sign, for computing, it is in the form of a 1D array.
 * @param pOut_Pred_Var [pointer] The variance of the predicted output data, Kpp - Kpt * Inv(Ktt + sigma^2 * I) * Ktp, for computing, it is in the form of a 1D array.
 *
 * //@param Ktt The kernel matrix Ktt, for computing, it is in the form of a 2D array.
 * @param Kttn The noised kernel matrix Kttn, for computing, it is in the form of a 2D array.
 * @param Kpt The kernel matrix Kpt, for computing, it is in the form of a 2D array.
 * @param Ktp The kernel matrix Ktp, for computing, it is in the form of a 2D array.
 * @param Kpp The kernel matrix Kpp, for computing, it is in the form of a 2D array.
 * @param IKttn The inverse of the noised kernel matrix Kttn, for computing, it is in the form of a 2D array.
 * @param KptxIKttn The product of Kpt and IKttn, for computing, it is in the form of a 2D array.
 * //@param In_Train_Mean The mean of the input data for training, for computing.
 * @param Out_Train_Mean The mean of the output data for training, for computing.
 * //@param In_Pred_Mean The mean of the input data for prediction, for computing.
 * @param Out_GT_Mean The mean of the ground truth output data for prediction, for computing.
 * @param Diff_Out_Train_Mean The difference between OUT_TRAIN and OUT_TRAIN_AVG, for computing.
 * @param Out_Pred_T1 The predicted output Term 1, Kpt * Inv(Ktt + sigma^2 * I) * (y_train - mean(y_train)), for computing.
 * @param Out_Pred_Mean The mean of the predicted output data, for computing.
 * @param Out_Pred The predicted output data, for computing.
 * @param Out_Pred_Var_T2 The variance of the predicted output data, Kpt * Inv(Ktt + sigma^2 * I) * Ktp, careful with the sign, for computing.
 * @param Out_Pred_Var The variance of the predicted output data, Kpp - Kpt * Inv(Ktt + sigma^2 * I) * Ktp, for computing.
 *
 */

// static float32_t IN_TRAIN[NUM_GPR_TRAIN][DIM_GPR_IN];
// static float32_t OUT_TRAIN[NUM_GPR_TRAIN][DIM_GPR_OUT];
// static float32_t OUT_TRAIN_MEAN[NUM_GPR_TRAIN][DIM_GPR_OUT];
static float32_t IN_PRED[NUM_GPR_PRED][DIM_GPR_IN];
static float32_t OUT_GT[NUM_GPR_PRED][DIM_GPR_OUT];
static float32_t OUT_PRED[NUM_GPR_PRED][DIM_GPR_OUT];
// static float32_t IN_TRAIN_AVG[DIM_GPR_IN];
static float32_t OUT_TRAIN_AVG[DIM_GPR_OUT];
// static float32_t IN_PRED_AVG[DIM_GPR_IN];
static float32_t OUT_GT_AVG[DIM_GPR_OUT];
static float32_t OUT_PRED_VAR_ORIGIN[NUM_GPR_PRED][NUM_GPR_PRED];
static float32_t OUT_PRED_VAR[NUM_GPR_PRED][DIM_GPR_OUT];
static float32_t OUT_PRED_STD[NUM_GPR_PRED][DIM_GPR_OUT];

static float32_t IN_TRAIN[NUM_GPR_TRAIN][DIM_GPR_IN] = {
    {-0.027153818, 0.001595143},
    {0.459352094, 0.001675976},
    {-1.393112726, 0.00309443},
    {1.273313908, 0.003114154},
    {-0.625930326, 0.005071804},
    {1.039416835, 0.00602996},
    {-1.411824491, 0.00620905},
    {-2.450327496, 0.006524486},
    {1.273313908, 0.006843167},
    {-1.701856862, 0.007515177},
    {-0.111356765, 0.009030439},
    {-0.588506794, 0.009148685},
    {-0.841115633, 0.009595511},
    {-0.176847945, 0.010167882},
    {1.114263899, 0.010216456},
    {-1.205995067, 0.010282116},
    {-0.494947965, 0.011098496},
    {1.24524626, 0.011249717},
    {1.24524626, 0.011326811},
    {0.188031489, 0.011673594},
    {-0.625930326, 0.012045779},
    {-0.176847945, 0.012064259},
    {-1.683145096, 0.012360465},
    {0.86165506, 0.012372318},
    {-1.842195106, 0.012419119},
    {-1.851550989, 0.012601391},
    {-1.411824491, 0.012636194},
    {-3.544965799, 0.012663324},
    {0.09447266, 0.013029125},
    {1.572702162, 0.013241147},
    {-0.326542072, 0.013251486},
    {0.38450503, 0.013380978},
    {0.047693245, 0.013386735},
    {-0.195559711, 0.013429773},
    {-0.055221467, 0.013449016},
    {0.375149147, 0.013451535},
    {1.114263899, 0.01345313},
    {0.075760894, 0.01351165},
    {-0.513659731, 0.013922766},
    {-0.073933233, 0.014045208},
    // {1.385584503, 0.014112076},
    // {-0.242339126, 0.014121063},
    // {0.552910923, 0.014188944},
    // {0.47806386, 0.014235425},
    // {0.169319723, 0.014308413},
    // {0.683893284, 0.014344904},
    // {-0.28911854, 0.014386385},
    // {-2.478395145, 0.014414233},
    // {1.08619625, 0.014478789},
    // {0.796163879, 0.01453814},
    // {0.749384465, 0.014563817},
    // {0.197387372, 0.014575255},
    // {0.983281538, 0.01461482},
    // {0.76809623, 0.01471572},
    // {0.262878552, 0.014752736},
    // {-0.934674462, 0.014929259},
    // {0.047693245, 0.014959371},
    // {0.225455021, 0.01518825},
    // {1.441719801, 0.0152185},
    // {-0.504303848, 0.015224292},
    // {0.665181518, 0.015258707},
    // {-1.411824491, 0.015261546},
    // {1.441719801, 0.015292676},
    // {-0.625930326, 0.015331216},
    // {-0.242339126, 0.015350209},
    // {-0.466880316, 0.015474852},
    // {-0.635286209, 0.015549892},
    // {1.984361011, 0.015621717},
    // {-0.644642091, 0.015633196},
    // {0.412572679, 0.015763233},
    // {-0.588506794, 0.015830669},
    // {-0.653997974, 0.015887896},
    // {-0.261050892, 0.015948383},
    // {-0.494947965, 0.016200905},
    // {-0.261050892, 0.016239515},
    // {-2.721648101, 0.016280185},
    // {1.432363918, 0.016520621},
    // {-0.635286209, 0.016657576},
    // {0.674537401, 0.016666331},
    // {0.880366825, 0.016767117},
    // {-2.122871594, 0.016786943},
    // {-0.635286209, 0.016812061},
    // {-0.139424413, 0.016865963},
    // {0.487419743, 0.016875853},
    // {-0.523015613, 0.016884811},
    // {1.479143333, 0.017108896},
    // {1.001993304, 0.01711305},
    // {0.721316816, 0.017139515},
    // {0.15996384, 0.017156965},
    // {-1.243418599, 0.017161445},
    // {-0.466880316, 0.017274542},
    // {-0.457524433, 0.017311457},
    // {-0.700777389, 0.017336263},
    // {0.871010943, 0.017394101},
    // {-0.392033252, 0.017432875},
    // {-0.934674462, 0.017435214},
    // {-1.224706833, 0.017493533},
    // {-0.494947965, 0.017706978},
    // {1.413652152, 0.017800153},
    // {0.786807996, 0.018137317},
};

static float32_t OUT_TRAIN[NUM_GPR_TRAIN][DIM_GPR_OUT] = {
    {0.001541595},
    {0.00215062},
    {0.003653763},
    {0.003247532},
    {0.007727102},
    {0.011101346},
    {0.011380381},
    {0.012652774},
    {0.009443972},
    {0.017131984},
    {0.019265345},
    {0.012888739},
    {0.03086787},
    {0.018705655},
    {0.015266891},
    {0.020276315},
    {0.028664981},
    {0.019987955},
    {0.018841027},
    {0.017108256},
    {0.022313004},
    {0.022789165},
    {0.023946532},
    {0.018431911},
    {0.028873205},
    {0.036266196},
    {0.064454756},
    {0.028746338},
    {0.020199155},
    {0.034649386},
    {0.028880162},
    {0.022256003},
    {0.030355093},
    {0.020655782},
    {0.024493448},
    {0.020462472},
    {0.019182196},
    {0.025183835},
    {0.029290589},
    {0.019565619},
    // {0.02591549},
    // {0.0293456},
    // {0.050643816},
    // {0.032969043},
    // {0.033590924},
    // {0.036082196},
    // {0.036164895},
    // {0.033824637},
    // {0.02874239},
    // {0.034254801},
    // {0.022037037},
    // {0.031283661},
    // {0.022421643},
    // {0.021118556},
    // {0.046625561},
    // {0.037356677},
    // {0.042961048},
    // {0.027719943},
    // {0.03724632},
    // {0.032660425},
    // {0.047368944},
    // {0.035672881},
    // {0.026849896},
    // {0.029988701},
    // {0.043529507},
    // {0.031592539},
    // {0.029642456},
    // {0.033599372},
    // {0.038054761},
    // {0.033384319},
    // {0.02821473},
    // {0.035697772},
    // {0.02538154},
    // {0.022672933},
    // {0.036357951},
    // {0.053374774},
    // {0.037310546},
    // {0.033349353},
    // {0.021851511},
    // {0.038435645},
    // {0.041869418},
    // {0.037279114},
    // {0.052651142},
    // {0.035132435},
    // {0.042234553},
    // {0.036564757},
    // {0.052646079},
    // {0.048909614},
    // {0.025163687},
    // {0.04409819},
    // {0.044285232},
    // {0.027525284},
    // {0.038638111},
    // {0.045538392},
    // {0.042201583},
    // {0.042482954},
    // {0.037130349},
    // {0.044903774},
    // {0.037065683},
    // {0.03868109},
};

static float32_t IN_PRED[NUM_GPR_PRED][DIM_GPR_IN] = {
    {0.145733146, 0.012362515},
    {-0.238355177, 0.026000512},
    {-0.5767187, 0.011350016},
    {-0.64073342, 0.013015032},
    {0.502386588, 0.02337619},
    // {-0.009731175, 0.016646195},
    // {1.490042276, 0.014337783},
    // {1.325432995, 0.017031693},
    // {1.023649312, 0.014570659},
    // {0.69443075, 0.01315309},
    // {2.468553003, 0.01143425},
    // {0.383502108, 0.011105503},
    // {1.105953953, 0.009123002},
    // {0.630416029, 0.013885126},
    // {-0.54928382, 0.013735056},
    // {1.371157795, 0.020071312},
    // {-0.174340457, 0.018198574},
    // {-0.375529578, 0.014629675},
    // {-0.402964458, 0.009476196},
    // {-0.412109418, 0.011117818},
};

static float32_t OUT_GT[NUM_GPR_PRED][DIM_GPR_OUT] = {
    {0.025796536},
    {0.043424018},
    {0.025709951},
    {0.032642322},
    {0.040896857},
    // {0.041180904},
    // {0.026071818},
    // {0.027838171},
    // {0.025818542},
    // {0.03550642},
    // {0.021196996},
    // {0.025962996},
    // {0.017436175},
    // {0.034871954},
    // {0.036751908},
    // {0.045151864},
    // {0.048081592},
    // {0.028411211},
    // {0.015472749},
    // {0.027825038},
};

// static float32_t *pKtt = NULL;       // [NUM_GPR_TRAIN * NUM_GPR_TRAIN];
static float32_t *pKttn = NULL;      // [NUM_GPR_TRAIN * NUM_GPR_TRAIN]; n for noised [unchanged through the process]
static float32_t *pKpt = NULL;       // [NUM_GPR_PRED * NUM_GPR_TRAIN];
static float32_t *pKtp = NULL;       // [NUM_GPR_TRAIN * NUM_GPR_PRED];
static float32_t *pKpp = NULL;       // [NUM_GPR_PRED * NUM_GPR_PRED];
static float32_t *pIKttn = NULL;     // [NUM_GPR_TRAIN * NUM_GPR_TRAIN]; [unchanged through the process]
static float32_t *pKptxIKttn = NULL; // [NUM_GPR_PRED * NUM_GPR_TRAIN];
// static float32_t *pIn_Train_Mean = NULL; // [NUM_GPR_TRAIN * DIM_GPR_IN]; [unchanged through the process]
static float32_t *pOut_Train_Mean = NULL; // [NUM_GPR_TRAIN * DIM_GPR_OUT]; [unchanged through the process]
// static float32_t *pIn_Pred_Mean = NULL; // [NUM_GPR_PRED * DIM_GPR_IN]; [unchanged through the process]
static float32_t *pOut_GT_Mean = NULL;         // [NUM_GPR_PRED * DIM_GPR_OUT]; [unchanged through the process]
static float32_t *pDiff_Out_Train_Mean = NULL; // [NUM_GPR_TRAIN * DIM_GPR_OUT]; [unchanged through the process]
static float32_t *pOut_Pred_T1 = NULL;         // [NUM_GPR_PRED * DIM_GPR_OUT];
static float32_t *pOut_Pred_Mean = NULL;       // [NUM_GPR_PRED * DIM_GPR_OUT]; [unchanged through the process]
static float32_t *pOut_Pred = NULL;            // [NUM_GPR_PRED * DIM_GPR_OUT];
static float32_t *pOut_Pred_Var_T2 = NULL;     // [NUM_GPR_PRED * DIM_GPR_OUT];  Kpt * Inv(Ktt + sigma^2 * I) * Ktp, careful with the sign
static float32_t *pOut_Pred_Var = NULL;        // [NUM_GPR_PRED * DIM_GPR_OUT];  Kpp - Kpt * Inv(Ktt + sigma^2 * I) * Ktp

// arm_matrix_instance_f32 Ktt;       // corresponding to and defiend by pKtt [unchanged through the process]
arm_matrix_instance_f32 Kttn;      // corresponding to and defiend by pKttn [unchanged through the process]
arm_matrix_instance_f32 Kpt;       // corresponding to and defiend by pKpt
arm_matrix_instance_f32 Ktp;       // corresponding to and defiend by pKtp
arm_matrix_instance_f32 Kpp;       // corresponding to and defiend by pKpp
arm_matrix_instance_f32 IKttn;     // inverse of Kttn [unchanged through the process]
arm_matrix_instance_f32 KptxIKttn; // Kpt * IKttn
// arm_matrix_instance_f32 In_Train_Mean; // mean of IN_TRAIN, corresponding to and defiend by pIn_Train_Mean [unchanged through the process]
arm_matrix_instance_f32 Out_Train_Mean; // mean of OUT_TRAIN, corresponding to and defiend by pOut_Train_Mean [unchanged through the process]
// arm_matrix_instance_f32 In_Pred_Mean; // mean of IN_PRED, corresponding to and defiend by pIn_Pred_Mean
arm_matrix_instance_f32 Out_GT_Mean;         // mean of OUT_GT, corresponding to and defiend by pOut_GT_Mean
arm_matrix_instance_f32 Diff_Out_Train_Mean; // difference between OUT_TRAIN and OUT_TRAIN_AVG [unchanged through the process]
arm_matrix_instance_f32 Out_Pred_T1;         // predicted output Term 1, Kpt * Inv(Ktt + sigma^2 * I) * (y_train - mean(y_train))
arm_matrix_instance_f32 Out_Pred_Mean;       // mean of the predicted output data [unchanged through the process]
arm_matrix_instance_f32 Out_Pred;            // predicted output data
arm_matrix_instance_f32 Out_Pred_Var_T2;     // Kpt * Inv(Ktt + sigma^2 * I) * Ktp, careful with the sign
arm_matrix_instance_f32 Out_Pred_Var;        // Kpp - Kpt * Inv(Ktt + sigma^2 * I) * Ktp

/**
 * ! KERNEL FUNCTION RELATED VARIABLES
 * @param RBF_Amplitude The amplitude of the RBF kernel.
 * @param RBF_LengthScale The length scale of the RBF kernel.
 * @param RBF_Coef The coefficient of the RBF kernel.
 * @param Noise_Lvl The noise level of the RBF kernel.
 * @param AB_Diff The difference between two points.
 * @param K_Unit The kernel function value.
 *
 */
float32_t RBF_Amplitude = 1;
float32_t RBF_LenScale[DIM_GPR_IN] = {5, 0.2};
float32_t RBF_Coef = 0.5;
float32_t Noise_Lvl = 0.0004; // 0.000009; //0.1
float32_t AB_Diff[DIM_GPR_IN];
float32_t K_Unit;

int progress_report = 2;

/* STATEMENTS --------------------------------------------------- END*/

/* HELPER FUNCTIONS ------------------------------------------- START*/
/**
 * ! [verified]
 * @name GPR_Kernel
 * @brief The kernel function for Gaussian Process Regression.
 * @param PntA The input point A.
 * @param PntB The input point B.
 * @param RBF_Amplitude The amplitude of the RBF kernel.
 * @param RBF_LenScale The length scale of the RBF kernel.
 * @param RBF_Coef The coefficient of the RBF kernel.
 *
 */
float32_t GPR_Kernel(float32_t PntA[DIM_GPR_IN], float32_t PntB[DIM_GPR_IN], float32_t RBF_Amplitude, float32_t RBF_LenScale[DIM_GPR_IN], float32_t RBF_Coef)
{
    int ind_t;

    // calculate the difference between PntA and PntB
    K_Unit = 0;
    for (ind_t = 0; ind_t < DIM_GPR_IN; ind_t++)
    {
        AB_Diff[ind_t] = PntA[ind_t] - PntB[ind_t];
        K_Unit += AB_Diff[ind_t] * AB_Diff[ind_t] / RBF_LenScale[ind_t] / RBF_LenScale[ind_t];
    }

    // calculate the kernel function
    K_Unit = RBF_Amplitude * RBF_Amplitude * exp(-RBF_Coef * K_Unit);
    return K_Unit;
}

int LoadData(void)
{
    /* STATEMENTS ---------------------- START*/
    int i, j, k, cnt_line, ind_row, ind_col;
    float32_t tmp;
    /* STATEMENTS ------------------------ END*/

    printf("[GPR_SPC] : Loading data from SD card...\n");

    /* IN_TRAIN ------------------------ START*/
    // // open the file
    // ret = f_open(&file, "x_train.csv", FA_READ);
    // if (ret)
    // {
    //     printf("[GPR_SPC] : Error opening training input\n\r");
    //     return 1;
    // }

    // // read the file, input entries are in total NUM_GPR_TRAIN
    // cnt_line = 0;
    // while ((f_gets(ReadBuffer, ReadBufferSize, &file) != NULL) && cnt_line < NUM_GPR_TRAIN)
    // {
    //     // parse the line
    //     token = strtok(ReadBuffer, ",");
    //     ind_col = 0;
    //     while (token != NULL)
    //     {
    //         sscanf(token, "%f", &tmp);
    //         IN_TRAIN[cnt_line][ind_col] = tmp;
    //         token = strtok(NULL, ",");
    //         ind_col++;
    //     }
    //     cnt_line++;
    // }

    // // close the file
    // f_close(&file);

    // print IN_TRAIN
    printf("[GPR_SPC] : IN_TRAIN loaded! (currently hard coded)\n");
    for (i = 0; i < NUM_GPR_TRAIN; i++)
    {
        printf("IN_TRAIN[%3d] = ", i);
        for (j = 0; j < DIM_GPR_IN; j++)
        {
            printf("%9f, ", IN_TRAIN[i][j]);
        }
        printf("\n");
    }

    /* IN_TRAIN -------------------------- END*/

    /* OUT_TRAIN ------------------------ START*/

    // print OUT_TRAIN
    printf("[GPR_SPC] : OUT_TRAIN loaded! (currently hard coded)\n");
    for (i = 0; i < NUM_GPR_TRAIN; i++)
    {
        printf("OUT_TRAIN[%3d] = ", i);
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            printf("%9f, ", OUT_TRAIN[i][j]);
        }
        printf("\n");
    }

    /* OUT_TRAIN -------------------------- END*/

    /* IN_PRED ---------------------- START*/

    // print IN_PRED
    printf("[GPR_SPC] : IN_PRED loaded! (currently hard coded)\n");
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        printf("IN_PRED[%3d] = ", i);
        for (j = 0; j < DIM_GPR_IN; j++)
        {
            printf("%9f, ", IN_PRED[i][j]);
        }
        printf("\n");
    }

    /* IN_PRED ------------------------ END*/

    /* OUT_GT --------------------------- START*/

    // print OUT_GTL
    printf("[GPR_SPC] : OUT_GT loaded! (currently hard coded)\n");
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        printf("OUT_GT[%3d] = ", i);
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            printf("%9f, ", OUT_GT[i][j]);
        }
        printf("\n");
    }

    /* OUT_GT ----------------------------- END*/

    printf("[GPR_SPC] : Data loaded!\n");

    return 0;
}

int GPR_SPC_Computing(void)
{
    /* STATEMENTS ---------------------- START*/
    int i, j;

    /* STATEMENTS ------------------------ END*/

    /* PREPARATION --------------------- START*/
    // pKtt = (float32_t *)calloc(NUM_GPR_TRAIN * NUM_GPR_TRAIN, sizeof(float32_t));
    pKttn = (float32_t *)calloc(NUM_GPR_TRAIN * NUM_GPR_TRAIN, sizeof(float32_t));
    pKpt = (float32_t *)calloc(NUM_GPR_PRED * NUM_GPR_TRAIN, sizeof(float32_t));
    pKtp = (float32_t *)calloc(NUM_GPR_TRAIN * NUM_GPR_PRED, sizeof(float32_t));
    pKpp = (float32_t *)calloc(NUM_GPR_PRED * NUM_GPR_PRED, sizeof(float32_t));
    pIKttn = (float32_t *)calloc(NUM_GPR_TRAIN * NUM_GPR_TRAIN, sizeof(float32_t));
    pKptxIKttn = (float32_t *)calloc(NUM_GPR_PRED * NUM_GPR_TRAIN, sizeof(float32_t));
    // pIn_Train_Mean = (float32_t *)calloc(NUM_GPR_TRAIN * DIM_GPR_IN, sizeof(float32_t));
    pOut_Train_Mean = (float32_t *)calloc(NUM_GPR_TRAIN * DIM_GPR_OUT, sizeof(float32_t));
    // pIn_Pred_Mean = (float32_t *)calloc(NUM_GPR_PRED * DIM_GPR_IN, sizeof(float32_t));
    pOut_GT_Mean = (float32_t *)calloc(NUM_GPR_PRED * DIM_GPR_OUT, sizeof(float32_t));
    pDiff_Out_Train_Mean = (float32_t *)calloc(NUM_GPR_TRAIN * DIM_GPR_OUT, sizeof(float32_t));
    pOut_Pred_T1 = (float32_t *)calloc(NUM_GPR_PRED * DIM_GPR_OUT, sizeof(float32_t));
    pOut_Pred_Mean = (float32_t *)calloc(NUM_GPR_PRED * DIM_GPR_OUT, sizeof(float32_t));
    pOut_Pred = (float32_t *)calloc(NUM_GPR_PRED * DIM_GPR_OUT, sizeof(float32_t));
    pOut_Pred_Var_T2 = (float32_t *)calloc(NUM_GPR_PRED * NUM_GPR_PRED, sizeof(float32_t));
    pOut_Pred_Var = (float32_t *)calloc(NUM_GPR_PRED * NUM_GPR_PRED, sizeof(float32_t));
    /* PREPARATION ----------------------- END*/

    /* COMPUTATION --------------------- START*/

    // [Kernal Matrices]

    // [Kernal Matrices] - Kttn (Ktt + Noise_Lvl * I); Noise_Lvl = sigma ^ 2 <unchanged>
    for (i = 0; i < NUM_GPR_TRAIN; i++)
    {
        for (j = 0; j < NUM_GPR_TRAIN; j++)
        {
            K_Unit = GPR_Kernel(IN_TRAIN[i], IN_TRAIN[j], RBF_Amplitude, RBF_LenScale, RBF_Coef);
            // pKtt[i * NUM_GPR_TRAIN + j] = K_Unit;
            pKttn[i * NUM_GPR_TRAIN + j] = K_Unit + Noise_Lvl * (i == j);
        }
    }
    arm_mat_init_f32(&Kttn, NUM_GPR_TRAIN, NUM_GPR_TRAIN, pKttn);

    // [Kernal Matrices] - Kpt, Ktp
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < NUM_GPR_TRAIN; j++)
        {
            K_Unit = GPR_Kernel(IN_PRED[i], IN_TRAIN[j], RBF_Amplitude, RBF_LenScale, RBF_Coef);
            pKpt[i * NUM_GPR_TRAIN + j] = K_Unit;
            pKtp[j * NUM_GPR_PRED + i] = K_Unit;
        }
    }
    arm_mat_init_f32(&Kpt, NUM_GPR_PRED, NUM_GPR_TRAIN, pKpt);
    arm_mat_init_f32(&Ktp, NUM_GPR_TRAIN, NUM_GPR_PRED, pKtp);

    // [Kernal Matrices] - Kpp
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < NUM_GPR_PRED; j++)
        {
            pKpp[i * NUM_GPR_PRED + j] = GPR_Kernel(IN_PRED[i], IN_PRED[j], RBF_Amplitude, RBF_LenScale, RBF_Coef);
        }
    }
    arm_mat_init_f32(&Kpp, NUM_GPR_PRED, NUM_GPR_PRED, pKpp);

    // [Kernal Matrices] - IKttn, i.e., the inverse of Kttn
    arm_mat_init_f32(&IKttn, NUM_GPR_TRAIN, NUM_GPR_TRAIN, pIKttn);
    arm_mat_inverse_f32(&Kttn, &IKttn);

    //[Interim Matrices] - KptxIKttn
    arm_mat_init_f32(&KptxIKttn, NUM_GPR_PRED, NUM_GPR_TRAIN, pKptxIKttn);
    arm_mat_mult_f32(&Kpt, &IKttn, &KptxIKttn);

    // [Interim Matrices] - OUT_TRAIN_AVG
    for (j = 0; j < DIM_GPR_OUT; j++)
    {
        OUT_TRAIN_AVG[j] = 0;
        for (i = 0; i < NUM_GPR_TRAIN; i++)
        {
            OUT_TRAIN_AVG[j] += OUT_TRAIN[i][j];
        }
        OUT_TRAIN_AVG[j] /= NUM_GPR_TRAIN;
    }

    // [Interim Matrices] - *In_Train_Mean, Out_Train_Mean, *In_Pred_Mean, Out_GT_Mean: * means commented out, no use
    for (i = 0; i < NUM_GPR_TRAIN; i++)
    {
        // for (j = 0; j < DIM_GPR_IN; j++)
        // {
        //     pIn_Train_Mean[i * DIM_GPR_IN + j] = IN_TRAIN_AVG[j];
        // }
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            pOut_Train_Mean[i * DIM_GPR_OUT + j] = OUT_TRAIN_AVG[j];
        }
    }

    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        // for (j = 0; j < DIM_GPR_IN; j++)
        // {
        //     pIn_Pred_Mean[i * DIM_GPR_IN + j] = IN_PRED_AVG[j];
        // }
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            pOut_GT_Mean[i * DIM_GPR_OUT + j] = OUT_GT_AVG[j];
        }
    }

    // [Interim Matrices] - Out_Train_Mean, Out_GT_Mean
    // instance the mean matrix
    // arm_mat_init_f32(&In_Train_Mean, NUM_GPR_TRAIN, DIM_GPR_IN, pIn_Train_Mean);
    arm_mat_init_f32(&Out_Train_Mean, NUM_GPR_TRAIN, DIM_GPR_OUT, pOut_Train_Mean);
    // arm_mat_init_f32(&In_Pred_Mean, NUM_GPR_PRED, DIM_GPR_IN, pIn_Pred_Mean);
    arm_mat_init_f32(&Out_GT_Mean, NUM_GPR_PRED, DIM_GPR_OUT, pOut_GT_Mean);

    // [Interim Matrices] -  pDiff_Out_Train_Mean & Diff_Out_Train_Mean
    for (i = 0; i < NUM_GPR_TRAIN; i++)
    {
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            pDiff_Out_Train_Mean[i * DIM_GPR_OUT + j] = OUT_TRAIN[i][j] - OUT_TRAIN_AVG[j];
        }
    }

    // instance the difference matrix
    arm_mat_init_f32(&Diff_Out_Train_Mean, NUM_GPR_TRAIN, DIM_GPR_OUT, pDiff_Out_Train_Mean);

    // [Interim Matrices] -  pOut_Pred_T1 &Out_Pred_T1, i.e., Kpt * Inv(Ktt + sigma^2 * I) * (y_train - mean(y_train))
    arm_mat_init_f32(&Out_Pred_T1, NUM_GPR_PRED, DIM_GPR_OUT, pOut_Pred_T1);

    // calculate the predicted output Term 1
    arm_mat_mult_f32(&KptxIKttn, &Diff_Out_Train_Mean, &Out_Pred_T1);

    // [Interim Matrices] -  Out_Pred_Mean, we use OUT_TRAIN_AVG as the mean of the predicted output data
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            pOut_Pred_Mean[i * DIM_GPR_OUT + j] = OUT_TRAIN_AVG[j];
        }
    }

    arm_mat_init_f32(&Out_Pred_Mean, NUM_GPR_PRED, DIM_GPR_OUT, pOut_Pred_Mean);

    // [Desired Results] - Out_Pred, the predicted output data

    // init the matrix instance
    arm_mat_init_f32(&Out_Pred, NUM_GPR_PRED, DIM_GPR_OUT, pOut_Pred);

    // calculate the predicted output data
    arm_mat_add_f32(&Out_Pred_T1, &Out_Pred_Mean, &Out_Pred);

    // fill out OUT_PRED
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            OUT_PRED[i][j] = pOut_Pred[i * DIM_GPR_OUT + j];
        }
    }

    // print out the result OUT_PRED, all elements
    printf("[GPR_SPC] : OUT_PRED calculated.\n\r");

    printf("[GPR_SPC] : OUT_PRED:\n\r");
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            printf("%12f\t", OUT_PRED[i][j]);
        }
        printf("\n\r");
    }

    // [Interim Matrices] - Out_Pred_Var_T2, i.e., Kpt * Inv(Ktt + sigma^2 * I) * Ktp, careful with the sign

    // init the matrix instance
    arm_mat_init_f32(&Out_Pred_Var_T2, NUM_GPR_PRED, NUM_GPR_PRED, pOut_Pred_Var_T2);

    // calculate the predicted output variance Term 2
    // Kpt * Inv(Ktt + sigma^2 * I) * Ktp = Out_Pred_Var_T2
    arm_mat_mult_f32(&KptxIKttn, &Ktp, &Out_Pred_Var_T2);

    // [Interim Matrices] - Out_Pred_Var, i.e., Kpp - Kpt * Inv(Ktt + sigma^2 * I) * Ktp

    // init the matrix instance
    arm_mat_init_f32(&Out_Pred_Var, NUM_GPR_PRED, NUM_GPR_PRED, pOut_Pred_Var);

    // calculate the predicted output variance
    arm_mat_sub_f32(&Kpp, &Out_Pred_Var_T2, &Out_Pred_Var);

    // [Interim Matrices] - fill out OUT_PRED_VAR_ORIGIN
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < NUM_GPR_PRED; j++)
        {
            OUT_PRED_VAR_ORIGIN[i][j] = pOut_Pred_Var[i * DIM_GPR_OUT + j];
        }
    }

    // [Interim Matrices] - fill out OUT_PRED_VAR, take the diagonal elements
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            OUT_PRED_VAR[i][j] = OUT_PRED_VAR_ORIGIN[i][i]; // here, single column output, multi-dim to be done in future
        }
    }

    // print out the result OUT_PRED_VAR, all elements
    printf("[GPR_SPC] : OUT_PRED_VAR:\n\r");
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            printf("%12f\t", OUT_PRED_VAR[i][j]);
        }
        printf("\n\r");
    }

    // [Desired Results] - calculate the OUT_PRED_STD
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            OUT_PRED_STD[i][j] = sqrt(OUT_PRED_VAR[i][j]);
        }
    }

    // print out the result OUT_PRED_STD, all elements

    printf("[GPR_SPC] : OUT_PRED_STD:\n\r");
    for (i = 0; i < NUM_GPR_PRED; i++)
    {
        for (j = 0; j < DIM_GPR_OUT; j++)
        {
            printf("%12f\t", OUT_PRED_STD[i][j]);
        }
        printf("\n\r");
    }

    /* COMPUTATION ----------------------- END*/

    return 0;
}

/* HELPER FUNCTIONS --------------------------------------------- END*/

int GPRSPC(void)
{
    /* STATEMENTS ---------------------- START*/

    /* STATEMENTS ------------------------ END*/

    printf("[GPR_SPC] : GPR-SPC calculation for validation against computer!\n");

    /* PREPARATION --------------------- START*/

    // load the data - IN_TRAIN, OUT_TRAIN, IN_PRED, OUT_GT, currently replaced by hard-coded data
    if (LoadData())
    {
        printf("[GPR_SPC] : Error loading data...\n\r");
        return 1;
    }

    /* PREPARATION ----------------------- END*/

    /* COMPUTATION --------------------- START*/
    if (GPR_SPC_Computing())
    {
        printf("[GPR_SPC] : Error in GPR-SPC computing...\n\r");
        return 1;
    }

    /* COMPUTATION ----------------------- END*/

    /* FREE MEMORY ---------------------- START*/
    // free the memory
    // free(pKtt);
    free(pKttn);
    free(pKpt);
    free(pKtp);
    free(pKpp);
    free(pIKttn);
    free(pKptxIKttn);
    // free(pIn_Train_Mean);
    free(pOut_Train_Mean);
    // free(pIn_Pred_Mean);
    free(pOut_GT_Mean);
    free(pDiff_Out_Train_Mean);
    free(pOut_Pred_T1);
    free(pOut_Pred_Mean);
    free(pOut_Pred);
    free(pOut_Pred_Var_T2);
    free(pOut_Pred_Var);

    /* FREE MEMORY ------------------------ END*/

    printf("[GPR_SPC] : GPR-SPC calculation done!\n");

    return 0;
}
