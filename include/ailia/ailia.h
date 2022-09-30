/**
 * \~japanese
 * @file ailia.h
 * @brief 推論ライブラリ
 * @copyright AXELL CORPORATION, ax Inc.
 * @date 2021/11/22
 *
 * \~english
 * @file ailia.h
 * @brief inference library
 * @copyright AXELL CORPORATION, ax Inc.
 * @date Novemver 22, 2021
 */

#ifndef INCLUDED_AILIA
#define INCLUDED_AILIA

/* 呼び出し規約 */

#if defined(_M_JS)
	#include <emscripten.h>
	#define AILIA_API EMSCRIPTEN_KEEPALIVE
#elif defined(_WIN64) || defined(_M_X64) || defined(__amd64__) || defined(__x86_64__) || defined(__APPLE__) || defined(__ANDROID__) || defined(ANDROID) || defined(__linux__) || defined(NN_NINTENDO_SDK)
	#define AILIA_API
#else
	#define AILIA_API __stdcall
#endif

#include "ailia_call.h"

#ifdef __cplusplus
extern "C" {
#endif
	/****************************************************************
	* ライブラリ状態定義
	**/

	/**
	* \~japanese
	* @def AILIA_STATUS_SUCCESS
	* @brief 成功
	*
	* \~english
	* @def AILIA_STATUS_SUCCESS
	* @brief Successful
	*/
	#define AILIA_STATUS_SUCCESS                    (   0)
	/**
	* \~japanese
	* @def AILIA_STATUS_INVALID_ARGUMENT
	* @brief 引数が不正
	* @remark API呼び出し時の引数を確認してください。
	*
	* \~english
	* @def AILIA_STATUS_INVALID_ARGUMENT
	* @brief Incorrect argument
	* @remark Please check argument of called API.
	*/
	#define AILIA_STATUS_INVALID_ARGUMENT           (  -1)
	/**
	* \~japanese
	* @def AILIA_STATUS_ERROR_FILE_API
	* @brief ファイルアクセスに失敗した
	* @remark 指定したパスのファイルが存在するか、権限を確認してください。
	*
	* \~english
	* @def AILIA_STATUS_ERROR_FILE_API
	* @brief File access failed.
	* @remark Please check file is exist or not, and check access permission.
	*/
	#define AILIA_STATUS_ERROR_FILE_API             (  -2)
	/**
	* \~japanese
	* @def AILIA_STATUS_INVALID_VERSION
	* @brief 構造体バージョンが不正
	* @remark API呼び出し時に指定した構造体バージョンを確認し、正しい構造体バージョンを指定してください。
	*
	* \~english
	* @def AILIA_STATUS_INVALID_VERSION
	* @brief Incorrect struct version
	* @remark Please check struct version that passed with API and please pass correct struct version.
	*/
	#define AILIA_STATUS_INVALID_VERSION            (  -3)
	/**
	* \~japanese
	* @def AILIA_STATUS_BROKEN
	* @brief 壊れたファイルが渡された
	* @remark モデルファイルが破損していないかを確認し、正常なモデルを渡してください。
	*
	* \~english
	* @def AILIA_STATUS_BROKEN
	* @brief A corrupt file was passed.
	* @remark Please check model file are correct or not, and please pass correct model.
	*/
	#define AILIA_STATUS_BROKEN                     (  -4)
	/**
	* \~japanese
	* @def AILIA_STATUS_MEMORY_INSUFFICIENT
	* @brief メモリが不足している
	* @remark メインメモリやVRAMの空き容量を確保してからAPIを呼び出してください。
	*
	* \~english
	* @def AILIA_STATUS_MEMORY_INSUFFICIENT
	* @brief Insufficient memory
	* @remark Please check usage of main memory and VRAM. And please call API after free memory.
	*/
	#define AILIA_STATUS_MEMORY_INSUFFICIENT        (  -5)
	/**
	* \~japanese
	* @def AILIA_STATUS_THREAD_ERROR
	* @brief スレッドの作成に失敗した
	* @remark スレッド数などシステムリソースを確認し、リソースを開放してからAPIを呼び出してください。
	*
	* \~english
	* @def AILIA_STATUS_THREAD_ERROR
	* @brief Thread creation failed.
	* @remark Please check usage of system resource (e.g. thread). And please call API after release system  resources.
	*/
	#define AILIA_STATUS_THREAD_ERROR               (  -6)
	/**
	* \~japanese
	* @def AILIA_STATUS_INVALID_STATE
	* @brief ailia の内部状態が不正
	* @remark APIドキュメントを確認し、呼び出し手順が正しいかどうかを確認してください。
	*
	* \~english
	* @def AILIA_STATUS_INVALID_STATE
	* @brief The internal status of the ailia is incorrect.
	* @remark Please check API document and API call steps.
	*/
	#define AILIA_STATUS_INVALID_STATE              (  -7)
	/**
	* \~japanese
	* @def AILIA_STATUS_UNSUPPORT_NET
	* @brief 非対応のネットワーク
	* @remark Detectorなどのラッパー関数に非対応のモデルファイルが渡されました。ドキュメント記載のサポートモデルかどうかを確認してください。
	*
	* \~english
	* @def AILIA_STATUS_UNSUPPORT_NET
	* @brief Unsupported network
	* @remark Non supported model file was passed to wrapper functions (e.g. Detector). Please check document whether presented models are supported or not.
	*/
	#define AILIA_STATUS_UNSUPPORT_NET              (  -9)
	/**
	* \~japanese
	* @def AILIA_STATUS_INVALID_LAYER
	* @brief レイヤーの重みやパラメータ、入出力形状が不正
	* @remark モデル中のレイヤーのパラメーターなどが不正でした。ailiaGetErrorDetail() で詳細なエラーメッセージを確認し、モデルファイルが正しいかを確認してください。
	*
	* \~english
	* @def AILIA_STATUS_INVALID_LAYER
	* @brief Incorrect layer weight, parameter, or input or output shape
	* @remark The layer of model had incorrect parameter or so on. Please call ailiaGetErrorDetail() and check detail message. And, please check model.
	*/
	#define AILIA_STATUS_INVALID_LAYER              ( -10)
	/**
	* \~japanese
	* @def AILIA_STATUS_INVALID_PARAMINFO
	* @brief パラメータファイルの内容が不正
	* @remark 指定したパラメーターファイルが破損していないかどうかを確認してください。
	*
	* \~english
	* @def AILIA_STATUS_INVALID_PARAMINFO
	* @brief The content of the parameter file is invalid.
	* @remark Please check parameter file are correct or not.
	*/
	#define AILIA_STATUS_INVALID_PARAMINFO          ( -11)
	/**
	* \~japanese
	* @def AILIA_STATUS_NOT_FOUND
	* @brief 指定した要素が見つからなかった
	* @remark 指定した名前やindexの要素が見つかりませんでした。指定した要素がモデルに含まれているかを確認してください。
	*
	* \~english
	* @def AILIA_STATUS_NOT_FOUND
	* @brief The specified element was not found.
	* @remark The specified element of passed name/index was not found. Please check the element are exisit on model or not.
	*/
	#define AILIA_STATUS_NOT_FOUND                  ( -12)
	/**
	* \~japanese
	* @def AILIA_STATUS_GPU_UNSUPPORT_LAYER
	* @brief GPUで未対応のレイヤーパラメータが与えられた
	* @remark GPUで実行できないレイヤーやパラメーターが与えられました。モデルファイルが正しいかどうかを確認した上で、ドキュメント記載のサポート窓口までお問い合わせください。
	*
	* \~english
	* @def AILIA_STATUS_GPU_UNSUPPORT_LAYER
	* @brief A layer parameter not supported by the GPU was given.
	* @remark The layer or parameter that not supported by the GPU was given. Please check model file are correct or not and contact support desk that described on document.
	*/
	#define AILIA_STATUS_GPU_UNSUPPORT_LAYER        ( -13)
	/**
	* \~japanese
	* @def AILIA_STATUS_GPU_ERROR
	* @brief GPU上での処理中にエラー
	* @remark 最新のGPUドライバーを利用しているか、VRAMが不足していないかなどを確認してください。
	*
	* \~english
	* @def AILIA_STATUS_GPU_ERROR
	* @brief Error during processing on the GPU
	* @remark Please check the GPU driver are latest and VRAM are sufficient or not.
	*/
	#define AILIA_STATUS_GPU_ERROR                  ( -14)
	/**
	* \~japanese
	* @def AILIA_STATUS_UNIMPLEMENTED
	* @brief 未実装
	* @remark 指定した環境では未実装な機能が呼び出されました。エラー内容をドキュメント記載のサポート窓口までお問い合わせください。
	*
	* \~english
	* @def AILIA_STATUS_UNIMPLEMENTED
	* @brief Unimplemented error
	* @remark The called API are not available on current environment. Please contact support desk that described on document.
	*/
	#define AILIA_STATUS_UNIMPLEMENTED              ( -15)
	/**
	* \~japanese
	* @def AILIA_STATUS_PERMISSION_DENIED
	* @brief 許可されていない操作
	* @remark 暗号化モデルなどで許可されていないAPIが呼び出されました。モデルファイルを確認し、APIの呼び出しを変更してください。
	*
	* \~english
	* @def AILIA_STATUS_PERMISSION_DENIED
	* @brief Operation not allowed
	* @remark The called API are not allowed on this model (e.g. encrypted model are used.). Please check model file and change API call flow.
	*/
	#define AILIA_STATUS_PERMISSION_DENIED          ( -16)
	/**
	* \~japanese
	* @def AILIA_STATUS_EXPIRED
	* @brief モデルの有効期限切れ
	* @remark モデルファイルの有効期限が切れています。ailia_obfuscate_cを利用してモデルを再生成してください。
	*
	* \~english
	* @def AILIA_STATUS_EXPIRED
	* @brief Model Expired
	* @remark The model file are expired. Please re generate model with ailia_obfuscate_c.
	*/
	#define AILIA_STATUS_EXPIRED                    ( -17)
	/**
	* \~japanese
	* @def AILIA_STATUS_UNSETTLED_SHAPE
	* @brief 形状が未確定
	* @remark 出力形状などが不定形です。出力形状取得時に発生した場合は入力形状を指定した上で推論を行い、その後形状取得APIを呼び出してください。
	*
	* \~english
	* @def AILIA_STATUS_UNSETTLED_SHAPE
	* @brief The shape is not yet determined
	* @remark The shape (e.g. output shape) are not determined. When called API that to get output shape, please set input shape and execute inference, then call API that to get output shape.
	*/
	#define AILIA_STATUS_UNSETTLED_SHAPE            ( -18)
	/**
	* \~japanese
	* @def AILIA_STATUS_DATA_HIDDEN
	* @brief アプリケーションからは取得できない情報だった
	* @remark 最適化などで指定した要素が削除されています。情報を取得する場合、最適化を無効にしてから呼び出してください。
	*
	* \~english
	* @def AILIA_STATUS_DATA_HIDDEN
	* @brief The information was not available from the application
	* @remark The specified information was removed due to optimization. If you need the information, please disable optimization and call API.
	*/
	#define AILIA_STATUS_DATA_HIDDEN                ( -19)
	#define AILIA_STATUS_DATA_REMOVED               AILIA_STATUS_DATA_HIDDEN
	/**
	* \~japanese
	* @def AILIA_STATUS_LICENSE_NOT_FOUND
	* @brief 有効なライセンスが見つからない
	* @remark トライアル版を利用する場合はライセンスファイルが必要です。ライセンスファイルに関しましては、ドキュメント記載のサポート窓口までお問い合わせください。
	*
	* \~english
	* @def AILIA_STATUS_LICENSE_NOT_FOUND
	* @brief No valid license found
	* @remark The license file are required for trial version. Please contact support desk that described on document.
	*/
	#define AILIA_STATUS_LICENSE_NOT_FOUND          ( -20)
	/**
	* \~japanese
	* @def AILIA_STATUS_LICENSE_BROKEN
	* @brief ライセンスが壊れている
	* @remark トライアル版を利用する際に必要なライセンスファイルが破損しています。ライセンスファイルに関しましては、ドキュメント記載のサポート窓口までお問い合わせください。
	*
	* \~english
	* @def AILIA_STATUS_LICENSE_BROKEN
	* @brief License is broken
	* @remark The license file that are required for trial version are broken. Please contact support desk that described on document.
	*/
	#define AILIA_STATUS_LICENSE_BROKEN             ( -21)
	/**
	* \~japanese
	* @def AILIA_STATUS_LICENSE_EXPIRED
	* @brief ライセンスの有効期限切れ
	* @remark トライアル版を利用するのに必要なライセンスファイルの有効期限が切れています。ライセンスファイルに関しましては、ドキュメント記載のサポート窓口までお問い合わせください。
	*
	* \~english
	* @def AILIA_STATUS_LICENSE_EXPIRED
	* @brief License expired
	* @remark The license file that are required for trial version are expired. Please contact support desk that described on document.
	*
	*/
	#define AILIA_STATUS_LICENSE_EXPIRED            ( -22)
	/**
	* \~japanese
	* @def AILIA_STATUS_NDIMENSION_SHAPE
	* @brief 形状が5次元以上であることを示す
	* @remark 4次元までしか扱えない旧APIが呼び出されました。APIドキュメントを参照し、5次元以上が扱えるAPIを利用してください。
	*
	* \~english
	* @def AILIA_STATUS_NDIMENSION_SHAPE
	* @brief Dimension of shape is 5 or more.
	* @remark The called API are supported up to 4 dimension. Please replace API that described on API document.
	*/
	#define AILIA_STATUS_NDIMENSION_SHAPE           ( -23)
	/**
	* \~japanese
	* @def AILIA_STATUS_OTHER_ERROR
	* @brief 不明なエラー
	* @remark その他のエラーが発生しました。ailiaGetErrorDetail() で詳細なエラーメッセージを確認し、エラー内容をドキュメント記載のサポート窓口までお問い合わせください。
	*
	* \~english
	* @def AILIA_STATUS_OTHER_ERROR
	* @brief Unknown error
	* @remark The misc error has been occured. Please call ailiaGetErrorDetail() and check detail message. And, please contact support desk that described on document.
	*/
	#define AILIA_STATUS_OTHER_ERROR                (-128)

	/****************************************************************
	* ネットワークオブジェクトのインスタンス
	**/

	struct AILIANetwork;

	/****************************************************************
	* 形状情報
	**/

	#define AILIA_SHAPE_VERSION (1)

	typedef struct _AILIAShape {
		/**
		* \~japanese
		* X軸のサイズ
		*
		* \~english
		* Size along the X axis
		*/
		unsigned int x;
		/**
		* \~japanese
		* Y軸のサイズ
		*
		* \~english
		* Size along the Y axis
		*/
		unsigned int y;
		/**
		* \~japanese
		* Z軸のサイズ
		*
		* \~english
		* Size along the Z axis
		*/
		unsigned int z;
		/**
		* \~japanese
		* W軸のサイズ
		*
		* \~english
		* Size along the W axis
		*/
		unsigned int w;
		/**
		* \~japanese
		* 次元情報
		*
		* \~english
		* Dimension information
		*/
		unsigned int dim;
	}AILIAShape;

	/****************************************************************
	* スレッド数
	**/

	#define AILIA_MULTITHREAD_AUTO	(0)

	/****************************************************************
	* 推論実行環境自動設定
	**/

	#define AILIA_ENVIRONMENT_ID_AUTO	(-1)

	/****************************************************************
	* 推論API
	**/

	/**
	* \~japanese
	* @brief ネットワークオブジェクトを作成します。
	* @param net ネットワークオブジェクトポインタへのポインタ
	* @param env_id 計算に利用する推論実行環境のID( ailiaGetEnvironment() で取得)  \ref AILIA_ENVIRONMENT_ID_AUTO  にした場合は自動で選択する
	* @param num_thread スレッド数の上限(  \ref AILIA_MULTITHREAD_AUTO  にした場合は自動で設定)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   ネットワークオブジェクトを作成します。
	*   推論実行環境を自動にした場合はCPUモードになり、BLASが利用できる場合はBLASを利用します。
	*   なお、BLASを利用する場合num_threadは無視される場合があります。
	*
	* \~english
	* @brief Creates a network instance.
	* @param net A pointer to the network instance pointer
	* @param env_id The ID of the inference backend used for computation (obtained by  ailiaGetEnvironment() ). It is selected automatically if  \ref AILIA_ENVIRONMENT_ID_AUTO  is specified.
	* @param num_thread The upper limit on the number of threads (It is set automatically if  \ref AILIA_MULTITHREAD_AUTO  is specified.)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   Creates a network instance.
	*   If the inference backend is set to automatic, CPU mode is used, while if BLAS is available, it uses BLAS.
	*   Note that if BLAS is used, num_thread may be ignored.
	*/
	int AILIA_API ailiaCreate(struct AILIANetwork **net, int env_id, int num_thread);

	/**
	* \~japanese
	* @brief ネットワークオブジェクトを初期化します。(ファイルから読み込み)
	* @param net ネットワークオブジェクトポインタ
	* @param path prototxtファイルのパス名(MBSC or UTF16)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   ファイルから読み込み、ネットワークオブジェクトを初期化します。
	*
	* \~english
	* @brief Initializes the network instance. (Read from file)
	* @param net A network instance pointer
	* @param path The path name to the prototxt file (MBSC or UTF16)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function reads the network instance from a file and initializes it.
	*/
	int AILIA_API ailiaOpenStreamFileA(struct AILIANetwork *net, const char    *path);
	int AILIA_API ailiaOpenStreamFileW(struct AILIANetwork *net, const wchar_t *path);

	/**
	* \~japanese
	* @brief ネットワークオブジェクトを初期化します。(ユーザ定義ファイルアクセスコールバック)
	* @param net ネットワークオブジェクトポインタ
	* @param fopen_args  \ref AILIA_USER_API_FOPEN に通知される引数ポインタ
	* @param callback ユーザ定義ファイルアクセスコールバック関数構造体
	* @param version ファイルアクセスコールバック関数構造体のバージョン( \ref AILIA_FILE_CALLBACK_VERSION )
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   ファイルから読み込み、ネットワークオブジェクトを初期化します。
	*
	* \~english
	* @brief Initializes the network instance. (User-defined file access callback)
	* @param net A network instance pointer
	* @param fopen_args An argument pointer supplied by AILIA_USER_API_FOPEN
	* @param callback A struct for the user-defined file access callback function
	* @param version The version of the struct for the file access callback function ( \ref AILIA_FILE_CALLBACK_VERSION )
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function reads the network instance from a file and initializes it.
	*/
	int AILIA_API ailiaOpenStreamEx(struct AILIANetwork *net, const void *fopen_args, ailiaFileCallback callback, int version);

	/**
	* \~japanese
	* @brief ネットワークオブジェクトを初期化します。(メモリから読み込み)
	* @param net ネットワークオブジェクトポインタ
	* @param buf prototxtファイルのデータへのポインタ
	* @param buf_size prototxtファイルのデータサイズ
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   メモリから読み込み、ネットワークオブジェクトを初期化します。
	*
	* \~english
	* @brief Initializes the network instance. (Read from memory)
	* @param net A network instance pointer
	* @param buf A pointer to the data in the prototxt file
	* @param buf_size The data size of the prototxt file
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function reads the network instance from memory and initializes it.
	*/
	int AILIA_API ailiaOpenStreamMem(struct AILIANetwork *net, const void *buf, unsigned int buf_size);

	/**
	* \~japanese
	* @brief ネットワークオブジェクトに重み係数を読み込みます。(ファイルから読み込み)
	* @param net ネットワークオブジェクトポインタ
	* @param path protobuf/onnxファイルのパス名(MBSC or UTF16)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   ファイルからネットワークオブジェクトに重み係数を読み込みます。
	*
	* \~english
	* @brief Reads weights into a network instance. (Read from file)
	* @param net A network instance pointer
	* @param path The path name to the protobuf/onnx file (MBSC or UTF16)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function reads weights into the network instance from a file.
	*/
	int AILIA_API ailiaOpenWeightFileA(struct AILIANetwork *net, const char    *path);
	int AILIA_API ailiaOpenWeightFileW(struct AILIANetwork *net, const wchar_t *path);

	/**
	* \~japanese
	* @brief ネットワークオブジェクトに重み係数を読み込みます。(ユーザ定義ファイルアクセスコールバック)
	* @param net ネットワークオブジェクトポインタ
	* @param fopen_args  \ref AILIA_USER_API_FOPEN に通知される引数ポインタ
	* @param callback ユーザ定義ファイルアクセスコールバック関数構造体
	* @param version ファイルアクセスコールバック関数構造体のバージョン( \ref AILIA_FILE_CALLBACK_VERSION )
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   ファイルからネットワークオブジェクトに重み係数を読み込みます。
	*
	* \~english
	* @brief Reads weights into a network instance. (User-defined file access callback)
	* @param net A network instance pointer
	* @param fopen_args An argument pointer supplied by AILIA_USER_API_FOPEN
	* @param callback A struct for the user-defined file access callback function
	* @param version The version of the struct for the file access callback function ( \ref AILIA_FILE_CALLBACK_VERSION )
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function reads weights into the network instance from a file.
	*/
	int AILIA_API ailiaOpenWeightEx(struct AILIANetwork *net, const void *fopen_args, ailiaFileCallback callback, int version);

	/**
	* \~japanese
	* @brief ネットワークオブジェクトに重み係数を読み込みます。(メモリから読み込み)
	* @param net ネットワークオブジェクトポインタ
	* @param buf protobuf/onnxファイルのデータへのポインタ
	* @param buf_size protobuf/onnxファイルのデータサイズ
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   メモリからネットワークオブジェクトに重み係数を読み込みます。
	*
	* \~english
	* @brief Reads weights into a network instance. (Read from memory)
	* @param net A network instance pointer
	* @param buf A pointer to the data in the protobuf/onnx file
	* @param buf_size The data size of the protobuf/onnx file
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function reads weights into the network instance from memory.
	*/
	int AILIA_API ailiaOpenWeightMem(struct AILIANetwork *net, const void *buf, unsigned int buf_size);

	/**
	* \~japanese
	* @brief ネットワークオブジェクトを破棄します。
	* @param net ネットワークオブジェクトポインタ
	*
	* \~english
	* @brief It destroys the network instance.
	* @param net A network instance pointer
	*/
	void AILIA_API ailiaDestroy(struct AILIANetwork *net);

	/**
	* \~japanese
	* @brief 推論時の入力データの形状を変更します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape 入力データの形状情報
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   prototxtで定義されている入力形状を変更します。
	*   prototxtに記述されているランクと同じにする必要があります。
	*   なお、重み係数の形状が入力形状に依存しているなどによりエラーが返る場合があります。
	*   prototxtで定義されているランクが4次元未満の場合は未使用の要素に1を設定する必要があります。
	*   prototxtで定義されているランクが5次元以上の場合は ailiaSetInputShapeND() を利用してください。
	*
	* \~english
	* @brief Changes the shape of the input data during inference.
	* @param net A network instance pointer
	* @param shape Shape information for the input data
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function changes the input shape defined in prototxt.
	*   The shape must have the same rank as the one contained in prototxt.
	*   Note that an error may be returned if the weights are dependent on the input shapes, among other reasons.
	*   The dimension of shape that defined in prototxt is less than 4, the unused element must be set to 1.
	*   The dimension of shape that defined in prototxt has 5 or more, please use  ailiaSetInputShapeND() ().
	*/
	int AILIA_API ailiaSetInputShape(struct AILIANetwork *net, const AILIAShape* shape, unsigned int version);

	/**
	* \~japanese
	* @brief 推論時の入力データの形状を変更します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape 入力データの各次元の大きさの配列(dim-1, dim-2, ... ,1, 0)
	* @param dim shapeの次元
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   prototxtで定義されている入力形状を変更します。
	*   prototxtに記述されているランクと同じにする必要があります。
	*   なお、重み係数の形状が入力形状に依存しているなどによりエラーが返る場合があります。
	*
	* \~english
	* @brief Changes the shape of the input data during inference.
	* @param net A network instance pointer
	* @param shape An array of shape that contains size of each axis (dim-1, dim-2, ... ,1, 0)
	* @param dim The size of shape.
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function changes the input shape defined in prototxt.
	*   The shape must have the same rank as the one contained in prototxt.
	*   Note that an error may be returned if the weights are dependent on the input shapes, among other reasons.
	*/
	int AILIA_API ailiaSetInputShapeND(struct AILIANetwork* net, const unsigned int* shape, unsigned int dim);

	/**
	* \~japanese
	* @brief 推論時の入力データの形状を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape 入力データの形状情報
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、5次元以上の場合 \ref AILIA_STATUS_NDIMENSION_SHAPE 、
	*   形状の一部が未確定の場合 \ref AILIA_STATUS_UNSETTLED_SHAPE 、それ以外のエラーの場合はエラーコードを返す。
	* @details
	*   形状が5次元以上の場合は ailiaGetInputDim() 、 ailiaGetInputShapeND() を利用してください。
	*   形状の一部が未確定の場合、該当する次元の値は0となり、それ以外の次元の値は有効な値が格納されます。
	*
	* \~english
	* @brief Gets the shape of the input data during inference.
	* @param net A network instance pointer
	* @param shape Shape information for the input data
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS .
	*   If shape has 5 or more dimension, it returns  \ref AILIA_STATUS_NDIMENSION_SHAPE .
	*   And if shape is not seattled, it returns \ref AILIA_STATUS_UNSETTLED_SHAPE , or an error code otherwise.
	* @details
	*   When dimension of shape is 5 or more, please use  ailiaGetInputDim() and  ailiaGetInputShapeND().
	*   When shape is not settled, this function stores 0 at unsettled dimension and otherwise stores valid value.
	*/
	int AILIA_API ailiaGetInputShape(struct AILIANetwork *net, AILIAShape* shape, unsigned int version);

	/**
	* \~japanese
	* @brief 推論時の入力データの次元を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param dim 入力データの次元の格納先
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、それ以外のエラーの場合はエラーコードを返す。
	*
	* \~english
	* @brief Gets the dimension of the input data during inference.
	* @param net A network instance pointer
	* @param dim The storage location of the dimension
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetInputDim(struct AILIANetwork* net, unsigned int* dim);

	/**
	* \~japanese
	* @brief 推論時の入力データの形状を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape 入力データの各次元の大きさの格納先配列(dim-1, dim-2, ... ,1, 0順で格納)
	* @param dim shapeの次元
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、形状の一部が未確定の場合 \ref AILIA_STATUS_UNSETTLED_SHAPE 、
	*   それ以外のエラーの場合はエラーコードを返す。
	* @details
	*   形状の一部が未確定の場合、該当する次元の値は0となり、それ以外の次元の値は有効な値が格納されます。
	*
	* \~english
	* @brief Gets the shape of the input data during inference.
	* @param net A network instance pointer
	* @param shape The storage location of the shape array. (It stores dim-1, dim-2, ... ,1, 0 order.)
	* @param dim The size of shape
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS .
	*   And if shape is not seattled, it returns  \ref AILIA_STATUS_UNSETTLED_SHAPE , or an error code otherwise.
	* @details
	*   When shape is not settled, this function stores 0 at unsettled dimension and otherwise stores valid value.
	*/
	int AILIA_API ailiaGetInputShapeND(struct AILIANetwork* net, unsigned int* shape, unsigned int dim);

	/**
	* \~japanese
	* @brief 推論時の出力データの形状を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape 出力データの形状情報
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、5次元以上の場合 \ref AILIA_STATUS_NDIMENSION_SHAPE 、
	*   それ以外のエラーの場合エラーコードを返す。
	*   形状が5次元以上の場合は ailiaGetOutputDim() 、 ailiaGetOutputShapeND() を利用してください。
	*
	* \~english
	* @brief Gets the shape of the output data during inference.
	* @param net A network instance pointer
	* @param shape Shape information of the output data
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS .
	*   And if shape has 5 or more dimension, it returns  \ref AILIA_STATUS_NDIMENSION_SHAPE , or an error code otherwise.
	* @details
	*   When dimension of shape is 5 or more, please use  ailiaGetOutputDim() () and  ailiaGetOutputShapeND() ().
	*/
	int AILIA_API ailiaGetOutputShape(struct AILIANetwork *net, AILIAShape* shape, unsigned int version);

	/**
	* \~japanese
	* @brief 推論時の出力データの次元を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param dim 出力データの次元の格納先
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、それ以外のエラーの場合はエラーコードを返す。
	*
	* \~english
	* @brief Gets the dimension of the output data during inference.
	* @param net A network instance pointer
	* @param dim The storage location of the dimension
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetOutputDim(struct AILIANetwork* net, unsigned int* dim);

	/**
	* \~japanese
	* @brief 推論時の出力データの形状を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape 出力データの各次元の大きさの格納先配列(dim-1, dim-2, ... ,1, 0順で格納)
	* @param dim shapeの次元
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、それ以外のエラーの場合はエラーコードを返す。
	*
	* \~english
	* @brief Gets the shape of the output data during inference.
	* @param net A network instance pointer
	* @param shape The storage location of the shape array. (It stores dim-1, dim-2, ... ,1, 0 order.)
	* @param dim The size of shape
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetOutputShapeND(struct AILIANetwork* net, unsigned int* shape, unsigned int dim);


	/**
	* \~japanese
	* @brief 推論を行い推論結果を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param dest 推論結果の書き出し先バッファにX,Y,Z,Wの順でnumeric型で格納  サイズはネットファイルのoutputSizeとなる
	* @param dest_size 推論結果の書き出し先バッファのbyte数
	* @param src 推論データ X,Y,Z,Wの順でnumeric型で格納 サイズはネットファイルのinputSizeとなる
	* @param src_size 推論データのbyte数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Performs the inferences and provides the inference result.
	* @param net A network instance pointer
	* @param dest The result is stored in the inference result destination buffer as numeric type data in the order of X, Y, Z, and W. The buffer has the same size as the network file outputSize.
	* @param dest_size The number of bytes for the destination buffer for the inference result
	* @param src The input is stored as numeric type data in the order of the inference data X, Y, Z, and W. The input has the same size as the network file inputSize.
	* @param src_size The number of bytes of the inference data
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaPredict(struct AILIANetwork *net, void * dest, unsigned int dest_size, const void *src, unsigned int src_size);

	/****************************************************************
	* 状態取得API
	**/

	/**
	* \~japanese
	* @brief 推論時の内部データ(Blob)の数を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param blob_count blobの数の格納先
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the amount of internal data (blob) during inference.
	* @param net A network instance pointer
	* @param blob_count Storage location of the number of blobs
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetBlobCount(struct AILIANetwork *net, unsigned int *blob_count);

	/**
	* \~japanese
	* @brief 推論時の内部データ(Blob)の形状を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape データの形状情報の格納先
	* @param blob_idx blobのインデックス (0～ ailiaGetBlobCount() -1)
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、5次元以上の場合 \ref AILIA_STATUS_NDIMENSION_SHAPE 、
	*   それ以外のエラーの場合はエラーコードを返す。
	* @details
	*   形状が5次元以上の場合は ailiaGetBlobDim() 、 ailiaGetBlobShapeND() を利用してください。
	*
	* \~english
	* @brief Gets the shape of the internal data (blob) during inference.
	* @param net A network instance pointer
	* @param shape Storage location of the data shape information
	* @param blob_idx The index of the blob (0 to  ailiaGetBlobCount() -1)
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS .
	*   And if shape has 5 or more dimension, it returns  \ref AILIA_STATUS_NDIMENSION_SHAPE , or an error code otherwise.
	* @details
	*   When dimension of shape is 5 or more, please use  ailiaGetBlobDim() and  ailiaGetBlobShapeND().
	*/
	int AILIA_API ailiaGetBlobShape(struct AILIANetwork *net, AILIAShape* shape, unsigned int blob_idx, unsigned int version);

	/**
	* \~japanese
	* @brief 推論時の内部データ(Blob)の次元を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param dim blobの次元の格納先
	* @param blob_idx blobのインデックス (0～ ailiaGetBlobCount() -1)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、それ以外のエラーの場合はエラーコードを返す。
	*
	* \~english
	* @brief Gets the dimension of the internal data (blob) during inference.
	* @param net A network instance pointer
	* @param dim The storage location of the dimension
	* @param blob_idx The index of the blob (0 to  ailiaGetBlobCount() -1)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetBlobDim(struct AILIANetwork* net, unsigned int* dim, unsigned int blob_idx);

	/**
	* \~japanese
	* @brief 推論時の内部データ(Blob)の形状を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape blobの各次元の大きさの格納先配列(dim-1, dim-2, ... ,1, 0順で格納)
	* @param dim shapeの次元
	* @param blob_idx blobのインデックス (0～ ailiaGetBlobCount() -1)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、それ以外のエラーの場合はエラーコードを返す。
	*
	* \~english
	* @brief Gets the amount of internal data (blob) during inference.
	* @param net A network instance pointer
	* @param shape The storage location of the shape array. (It stores dim-1, dim-2, ... ,1, 0 order.)
	* @param dim The size of shape
	* @param blob_idx The index of the blob (0 to  ailiaGetBlobCount() -1)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetBlobShapeND(struct AILIANetwork* net, unsigned int* shape, unsigned int dim, unsigned int blob_idx);

	/**
	* \~japanese
	* @brief 推論時の内部データ(Blob)を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param dest 推論結果の書き出し先バッファにX,Y,Z,Wの順でnumeric型で格納
	* @param dest_size 推論結果の書き出し先バッファのbyte数
	* @param blob_idx blobのインデックス (0～ ailiaGetBlobCount() -1)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*    ailiaPredict() または ailiaUpdate() を一度も実行していない場合は \ref AILIA_STATUS_INVALID_STATE が返ります。
	*
	* \~english
	* @brief Gets the internal data (blob) during inference.
	* @param net A network instance pointer
	* @param dest The result is stored in the inference result destination buffer as numeric type data in the order of X, Y, Z, and W.
	* @param dest_size The number of bytes for the inference result destination buffer
	* @param blob_idx The index of the blob (0 to  ailiaGetBlobCount() -1)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   If  ailiaPredict()  or  ailiaUpdate()  is not run at all, the function returns  \ref AILIA_STATUS_INVALID_STATE .
	*/
	int AILIA_API ailiaGetBlobData(struct AILIANetwork *net, void* dest, unsigned int dest_size, unsigned int blob_idx);

	/**
	* \~japanese
	* @brief 推論時の内部データ(Blob)のインデックスを名前で探し取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param blob_idx blobのインデックス (0～ ailiaGetBlobCount() -1)
	* @param name 検索するBlob名
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Searches by name for the index of the internal data (blob) during inference and returns it.
	* @param net A network instance pointer
	* @param blob_idx The index of the blob (0 to  ailiaGetBlobCount() -1)
	* @param name The name of the blob to search for
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaFindBlobIndexByName(struct AILIANetwork *net, unsigned int* blob_idx, const char* name);

	/**
	* \~japanese
	* @brief 内部データ(Blob)の名前の出力に必要なバッファのサイズを取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param blob_idx blobのインデックス (0～ ailiaGetBlobCount() -1)
	* @param buffer_size Blob名の出力に必要なバッファのサイズ(終端null文字分を含む)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the size of the buffer needed for output of the name of the internal data (blob).
	* @param net A network instance pointer
	* @param blob_idx The index of the blob (0 to  ailiaGetBlobCount() -1)
	* @param buffer_size The size of the buffer needed for output of the blob name (including the null terminator)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetBlobNameLengthByIndex(struct AILIANetwork *net, const unsigned int blob_idx, unsigned int* buffer_size);

	/**
	* \~japanese
	* @brief 推論時の内部データ(Blob)の名前をインデックスで探し取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param buffer Blob名の出力先バッファ
	* @param buffer_size バッファのサイズ(終端null文字分を含む)
	* @param blob_idx 検索するblobのインデックス
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Searches by index for the name of the internal data (blob) during inference and returns it.
	* @param net A network instance pointer
	* @param buffer The output destination buffer for the blob name
	* @param buffer_size The size of the buffer (including the null terminator)
	* @param blob_idx The index of the blob to search for
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaFindBlobNameByIndex(struct AILIANetwork *net, char* buffer, const unsigned int buffer_size, const unsigned int blob_idx);

	/**
	* \~japanese
	* @brief ネットワークSummary用に必要なバッファのサイズを取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param buffer_size バッファのサイズの格納先(終端null文字分を含む)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the size of the buffer needed for the network summary.
	* @param net A network instance pointer
	* @param buffer_size The storage location of the buffer size (including the null terminator)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetSummaryLength(struct AILIANetwork * net, unsigned int * buffer_size);

	/**
	* \~japanese
	* @brief 各Blobの名前と形状を表示します。
	* @param net ネットワークオブジェクトポインタ
	* @param buffer Summaryの出力先
	* @param buffer_size 出力バッファのサイズ(終端null文字分を含む)。 ailiaGetSummaryLength() で取得した値を設定してください。
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Shows the name and shape of each blob.
	* @param net A network instance pointer
	* @param buffer The output destination of the summary
	* @param buffer_size The size of the output buffer (including the null terminator). Set the value obtained by  ailiaGetSummaryLength() .
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaSummary(struct AILIANetwork * net, char* const buffer, const unsigned int buffer_size);

	/****************************************************************
	* 複数入力指定・推論API
	**/

	/**
	* \~japanese
	* @brief 入力データ(Blob)の数を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param input_blob_count 入力blobの数の格納先
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Get the number of input data blobs.
	* @param net A network instance pointer
	* @param input_blob_count Storage location of the number of input blobs
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetInputBlobCount(struct AILIANetwork *net, unsigned int *input_blob_count);

	/**
	* \~japanese
	* @brief 入力データ(Blob)のインデックスを取得します
	* @param net ネットワークオブジェクトポインタ
	* @param blob_idx blobのインデックス(0～ ailiaGetBlobCount() -1)
	* @param input_blob_idx 入力blob内でのインデックス(0～ ailiaGetInputBlobCount() -1)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Get the blob index of the input data.
	* @param net A network instance pointer
	* @param blob_idx index of the blob (between 0 and  ailiaGetBlobCount() -1)
	* @param input_blob_idx index among the input blobs (between 0 and  ailiaGetInputBlobCount() -1)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetBlobIndexByInputIndex(struct AILIANetwork *net, unsigned int *blob_idx, unsigned int input_blob_idx);

	/**
	* \~japanese
	* @brief 指定したBlobに入力データを与えます。
	* @param net ネットワークオブジェクトポインタ
	* @param src 推論データ X,Y,Z,Wの順でnumeric型で格納
	* @param src_size 推論データのbyte数
	* @param blob_idx 入力するblobのインデックス
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   複数入力があるネットワークなどで入力を指定する場合に用います。
	*   blob_idxで入力レイヤーのblob以外のものを指定した場合、 \ref AILIA_STATUS_INVALID_ARGUMENT が返ります。
	*
	* \~english
	* @brief Provides the specified blob with the input data.
	* @param net A network instance pointer
	* @param src The inference data is stored as numeric type data in the order of X, Y, Z, and W.
	* @param src_size The number of bytes of the inference data
	* @param blob_idx The index of the blob for input
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function is used to specify the input on networks with multiple inputs.
	*   If something other than a blob in the input layer is specified for blob_idx, the function returns  \ref AILIA_STATUS_INVALID_ARGUMENT .
	*/
	int AILIA_API ailiaSetInputBlobData(struct AILIANetwork *net, const void* src, unsigned int src_size, unsigned int blob_idx);

	/**
	* \~japanese
	* @brief 指定したBlobの形状を変更します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape 入力データの形状情報
	* @param blob_idx 変更するblobのインデックス
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   複数入力があるネットワークなどで入力形状を変更する場合に用います。
	*   blob_idxは入力レイヤーのblob以外のものを指定した場合 \ref AILIA_STATUS_INVALID_ARGUMENT が返ります。
	*   その他の注意点は ailiaSetInputShape() の解説を参照してください。
	*   入力形状のランクが5次元以上の場合は ailiaSetInputBlobShapeND() を利用してください。
	*
	* \~english
	* @brief Change the shape of the blob given by its index
	* @param net network object pointer
	* @param shape new shape of the blob
	* @param blob_idx index referencing the blob to reshape
	* @param version AILIA_SHAPE_VERSION
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and otherwise the coresponding error code.
	* @details
	*   This is useful to change the network input shape in a context where there are several input blobs.
	*   If blob_idx does not correspond to an input layer,  \ref AILIA_STATUS_INVALID_ARGUMENT  is returned.
	*   For other related remarks, see the documentation of  ailiaSetInputShape().
	*   If dimension of shape has 5 or more, please use  ailiaSetInputBlobShapeND().
	*/
	int AILIA_API ailiaSetInputBlobShape(struct AILIANetwork *net, const AILIAShape* shape, unsigned int blob_idx, unsigned int version);


	/**
	* \~japanese
	* @brief 指定したBlobの形状を変更します。
	* @param net ネットワークオブジェクトポインタ
	* @param shape 入力データの各次元の大きさの配列(dim-1, dim-2, ... ,1, 0)
	* @param dim shapeの次元
	* @param blob_idx 変更するblobのインデックス
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   複数入力があるネットワークなどで入力形状を変更する場合に用います。
	*   blob_idxは入力レイヤーのblob以外のものを指定した場合 \ref AILIA_STATUS_INVALID_ARGUMENT が返ります。
	*   その他の注意点は ailiaSetInputShapeND() の解説を参照してください。
	*
	* \~english
	* @brief Change the shape of the blob given by its index
	* @param net network object pointer
	* @param shape An array of shape that contains size of each axis (dim-1, dim-2, ... ,1, 0)
	* @param dim The size of shape.
	* @param blob_idx index referencing the blob to reshape
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and otherwise the coresponding error code.
	* @details
	*   This is useful to change the network input shape in a context where there are several input blobs.
	*   If blob_idx does not correspond to an input layer,  \ref AILIA_STATUS_INVALID_ARGUMENT  is returned.
	*   For other related remarks, see the documentation of  ailiaSetInputShapeND().
	*/
	int AILIA_API ailiaSetInputBlobShapeND(struct AILIANetwork* net, const unsigned* shape, const unsigned dim, unsigned int blob_idx);

	/**
	* \~japanese
	* @brief 事前に指定した入力データで推論を行います。
	* @param net ネットワークオブジェクトポインタ
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*    ailiaSetInputBlobData() を用いて入力を与えた場合などに用います。
	*   推論結果は ailiaGetBlobData() で取得してください。
	*
	* \~english
	* @brief Makes inferences with the input data specified in advance.
	* @param net A network instance pointer
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function is used when, for example, the input is provided with  ailiaSetInputBlobData() .
	*   Get the inference result with  ailiaGetBlobData() .
	*/
	int AILIA_API ailiaUpdate(struct AILIANetwork *net);

	/**
	* \~japanese
	* @brief 出力データ(Blob)の数を取得します。
	* @param net ネットワークオブジェクトポインタ
	* @param output_blob_count 出力blobの数の格納先
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Get the number of output data blobs.
	* @param net A network instance pointer
	* @param output_blob_count Storage location for the number of output blobs.
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetOutputBlobCount(struct AILIANetwork *net, unsigned int *output_blob_count);

	/**
	* \~japanese
	* @brief 出力データ(Blob)のインデックスを取得します
	* @param net ネットワークオブジェクトポインタ
	* @param blob_idx blobのインデックス(0～ ailiaGetBlobCount() -1)
	* @param output_blob_idx 出力blob内でのインデックス(0～ ailiaGetOutputBlobCount() -1)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Get the blob index of the input data blob.
	* @param net A network instance pointer
	* @param blob_idx blob index (between 0 and  ailiaGetBlobCount() -1)
	* @param output_blob_idx index among output blobs (between 0 and  ailiaGetOutputBlobCount() -1)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetBlobIndexByOutputIndex(struct AILIANetwork *net, unsigned int *blob_idx, unsigned int output_blob_idx);

	/****************************************************************
	* 推論実行環境取得・指定API
	**/

	#define  AILIA_ENVIRONMENT_VERSION              (2)

	#define  AILIA_ENVIRONMENT_TYPE_CPU             (0)
	#define  AILIA_ENVIRONMENT_TYPE_BLAS            (1)
	#define  AILIA_ENVIRONMENT_TYPE_GPU             (2)
	#define  AILIA_ENVIRONMENT_TYPE_REMOTE          (3)

	#define AILIA_ENVIRONMENT_BACKEND_NONE          (0)
	#define AILIA_ENVIRONMENT_BACKEND_AMP           (1)
	#define AILIA_ENVIRONMENT_BACKEND_CUDA          (2)
	#define AILIA_ENVIRONMENT_BACKEND_MPS           (3)
	#define AILIA_ENVIRONMENT_BACKEND_VULKAN        (6)

	#define AILIA_ENVIRONMENT_PROPERTY_NORMAL       (0)
	/**
	* \~japanese
	* 省電力なGPU(内蔵GPUなど)を用いることを示す(MPS用)
	*
	* \~english
	* Indicates that a low-power GPU (e.g. integrated GPU) will be preferentially used. (for MPS)
	*/
	#define AILIA_ENVIRONMENT_PROPERTY_LOWPOWER     (1)
	/**
	* \~japanese
	* FP16で動作することを示す
	*
	* \~english
	* Indicates that a FP16 mode
	*/
	#define AILIA_ENVIRONMENT_PROPERTY_FP16         (2)

	typedef struct _AILIAEnvironment {
		/**
		* \~japanese
		* 環境を識別するID( ailiaCreate() の引数に与える)
		*
		* \~english
		* The ID to identify the inference backend (passed to  ailiaCreate()  as an argument)
		*/
		int id;
		/**
		* \~japanese
		* 環境の種別( \ref AILIA_ENVIRONMENT_TYPE_CPU  or BLAS or GPU)
		*
		* \~english
		* The type of the inference backend ( \ref AILIA_ENVIRONMENT_TYPE_CPU , BLAS, or GPU)
		*/
		int type;
		/**
		* \~japanese
		* デバイス名(シングルトンで保持されており開放不要)(ASCII)
		*
		* \~english
		* The device name. It is valid until the AILIANetwork instance is destroyed.
		*/
		const char* name;
		/**
		* \~japanese
		* 環境のバックエンド (AILIA_ENVIRONMENT_BACKEND_*)
		*
		* \~english
		* Computational (hardware) backend enabled by this environment (AILIA_ENVIRONMENT_BACKEND_*)
		*/
		int backend;
		/**
		* \~japanese
		* 環境の特性などを示す(AILIA_ENVIRONMENT_PROPERTY_* の論理和)
		*
		* \~english
		* Additional property (low-power etc) of the environment (Logical-OR of AILIA_ENVIRONMENT_PROPERTY_*)
		*/
		int props;
	}AILIAEnvironment;

	/**
	* \~japanese
	* @brief 一時キャッシュディレクトリを指定します
	* @param cache_dir 一時キャッシュディレクトリ
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   指定したキャッシュディレクトリは推論実行環境毎に最適化したマシンコードを生成して保存するためにシステムが利用します。
	*   ailia の実行開始時に一度だけ呼び出してください。二回目以降の呼び出しに対しては無視して成功を返します。
	*   複数スレッドから呼び出された場合も内部で排他制御しているので特に問題は発生しません。
	*   Vulkan のシェーダーキャッシュ機能など、この API を呼ぶまで利用できないものがあります。
	*   cache_dirにはContext.getCacheDir()で取得したファイルパスを指定してください。
	*
	* \~english
	* @brief Specifies a temporary cache directory.
	* @param cache_dir Temporary cache directory
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This system uses the specified cache directory to generate and store machine code optimized for each inference backend.
	*   Call only once at the start of execution of ailia. It ignores any second and subsequent calls, and automatically returns success.
	*   There is no particular problem if it is called from multiple threads, as it provides exclusive control internally.
	*   Some functions, such as Vulkan shader cache, cannot be used until this API function is called.
	*   Specify the file path obtained by Context.getCacheDir() for cache_dir.
	*/
	int AILIA_API ailiaSetTemporaryCachePathA(const char    * cache_dir);
	int AILIA_API ailiaSetTemporaryCachePathW(const wchar_t * cache_dir);

	/**
	* \~japanese
	* @brief 利用可能な計算環境(CPU, GPU)の数を取得します
	* @param env_count 計算環境情報の数の格納先
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the number of available computational environments (CPU, GPU).
	* @param env_count The storage location of the number of computational environment information
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetEnvironmentCount(unsigned int * env_count);

	/**
	* \~japanese
	* @brief 計算環境の一覧を取得します
	* @param env 計算環境情報の格納先(AILIANetworkインスタンスを破棄するまで有効)
	* @param env_idx 計算環境情報のインデックス(0～ ailiaGetEnvironmentCount() -1)
	* @param version AILIA_ENVIRONMENT_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the list of computational environments.
	* @param env The storage location of the computational environment information (valid until the AILIANetwork instance is destroyed)
	* @param env_idx The index of the computational environment information (0 to  ailiaGetEnvironmentCount() -1)
	* @param version AILIA_ENVIRONMENT_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetEnvironment(AILIAEnvironment** env, unsigned int env_idx, unsigned int version);

	/**
	* \~japanese
	* @brief 選択された計算環境を取得します
	* @param net ネットワークオブジェクトポインタ
	* @param env 計算環境情報の格納先(AILIANetworkインスタンスを破棄するまで有効)
	* @param version AILIA_ENVIRONMENT_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the selected computational environment.
	* @param net A network instance pointer
	* @param env The storage location of the computational environment information (valid until the AILIANetwork instance is destroyed)
	* @param version AILIA_ENVIRONMENT_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaGetSelectedEnvironment(struct AILIANetwork *net, AILIAEnvironment** env, unsigned int version);

	/****************************************************************
	* メモリモードAPI
	**/

	/**
	* \~japanese
	* 中間バッファーの開放は行わない
	*
	* \~english
	* Do not release the intermediate buffer
	*/
	#define AILIA_MEMORY_NO_OPTIMIZATION                        (0)
	/**
	* \~japanese
	* 重みなどの定数となる中間バッファーを開放する
	*
	* \~english
	* Releases the intermediate buffer that is a constant such as weight
	*/
	#define AILIA_MEMORY_REDUCE_CONSTANT                        (1)
	/**
	* \~japanese
	* 入力指定のinitializerを変更不可にし、重みなどの定数となる中間バッファーを開放する
	*
	* \~english
	* Disable the input specified initializer and release the intermediate buffer that becomes a constant such as weight.
	*/
	#define AILIA_MEMORY_REDUCE_CONSTANT_WITH_INPUT_INITIALIZER (2)
	/**
	* \~japanese
	* 推論時の中間バッファーを開放する
	*
	* \~english
	* Release intermediate buffer during inference
	*/
	#define AILIA_MEMORY_REDUCE_INTERSTAGE                      (4)
	/**
	* \~japanese
	* 中間バッファーを共有して推論する。 \ref AILIA_MEMORY_REDUCE_INTERSTAGE と併用した場合、共有可能な中間バッファーは開放しない。
	*
	* \~english
	* Infer by sharing the intermediate buffer. When used with  \ref AILIA_MEMORY_REDUCE_INTERSTAGE , the sharable intermediate buffer is not opened.
	*/
	#define AILIA_MEMORY_REUSE_INTERSTAGE                       (8)

	#define AILIA_MEMORY_OPTIMAIZE_DEFAULT (AILIA_MEMORY_REDUCE_CONSTANT)

	/**
	* \~japanese
	* @brief 推論時のメモリの使用方針を設定します
	* @param net ネットワークオブジェクトポインタ
	* @param mode メモリモード(論理和で複数指定可) AILIA_MEMORY_XXX (デフォルト: \ref AILIA_MEMORY_REDUCE_CONSTANT )
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   メモリの使用方針を変更します。 \ref AILIA_MEMORY_NO_OPTIMIZATION 以外を指定した場合は、
	*   推論時に確保する中間バッファーを開放するため、推論時のメモリ使用量を削減することができます。
	*    ailiaCreate() の直後に指定する必要があります。ailiaOpenを呼び出した後は変更することができません。
	*   なお、中間バッファーを開放するように指定した場合、該当するBlobに対し、 ailiaGetBlobData() を呼び出すと
	*    \ref AILIA_STATUS_DATA_HIDDEN エラーが返ります。
	*
	* \~english
	* @brief Set the memory usage policy for inference
	* @param net A network instance pointer
	* @param mode Memory mode (Multiple specifications possible with logical sum) AILIA_MEMORY_XXX (Default : \ref AILIA_MEMORY_REDUCE_CONSTANT )
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   Change the memory usage policy.
	*   If a value other than  \ref AILIA_MEMORY_NO_OPTIMIZATION  is specified,
	*   the intermediate buffer secured during inference will be released, so the memory usage during inference can be reduced.
	*   Must be specified immediately after  ailiaCreate() . It cannot be changed after calling ailiaOpen.
	*   If you specify to release the intermediate buffer, calling  ailiaGetBlobData()  for the corresponding blob will return an  \ref AILIA_STATUS_DATA_HIDDEN  error.
	*/
	int AILIA_API ailiaSetMemoryMode(struct AILIANetwork* net, unsigned int mode);

	/****************************************************************
	* 最適化制御API
	**/

	/**
	* \~japanese
	* @brief 推論時のレイヤー統合を無効化します
	* @param net ネットワークオブジェクトポインタ
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   レイヤー統合により取得できなくなるBlobを取得する必要がある場合などに用います。
	*   ailiaCreate() の直後に指定する必要があります。ailiaOpenを呼び出した後は変更することができません。
	*   なお、レイヤー統合を無効化すると推論速度が低下する場合があります。
	*
	* \~english
	* @brief Disalbe layer fusion optimaization for inference
	* @param net A network instance pointer
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This api use to get blob that remove by layer fusion optimization.
	*   Must be specified immediately after  ailiaCreate() . It cannot be changed after calling ailiaOpen.
	*   Note: When disable layer fusion optimization, inference speed may be down.
	*/
	int AILIA_API ailiaDisableLayerFusion(struct AILIANetwork* net);

	/****************************************************************
	* プロファイルモードAPI
	**/

	/**
	* \~japanese
	* プロファイルモードを無効にします(デフォルト)
	*
	* \~english
	* Disable profile mode (Default)
	*/
	#define AILIA_PROFILE_DISABLE       (0x00)
	/**
	* \~japanese
	* レイヤー別の処理時間を計測します。複数回推論した場合は平均値が保存されます。
	*
	* \~english
	* Measure the processing time for each layer.
	* If you infer multiple times, the average value excluding the first execution is saved.
	*/
	#define AILIA_PROFILE_AVERAGE       (0x01)

	/**
	* \~japanese
	* @brief プロファイルモードをセットします
	* @param net          ネットワークオブジェクトポインタ
	* @param mode         プロファイルモード
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   プロファイルモードを指定します。デフォルトは無効です。
	*   ailiaOpenStreamXXXを呼び出したあとに呼び出してください。
	*   プロファイルモードを有効にした場合、 ailiaSummary() の出力にプロファイル結果が追加されます。
	*
	* \~english
	* @brief Set the profile mode.
	* @param net          The network instance pointer
	* @param mode         Profile mode AILIA_PROFILE_XXX (Default : \ref AILIA_PROFILE_DISABLE )
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   Set the profile mode. The default is profile disabled.
	*   Call it after calling ailiaOpenStreamXXX.
	*   When profile mode is enabled, you can get profile result via  ailiaSummary() .
	*/
	int AILIA_API ailiaSetProfileMode(struct AILIANetwork * net, unsigned int mode);

	/****************************************************************
	* 情報取得API
	**/

	/**
	* \~japanese
	* @brief ステータスコードに対応する文字列を返します。
	* @return
	*   ステータスコードに対応する文字列。
	* @details
	*   返値は解放する必要はありません。
	*   返された文字列は ailia のライブラリ(ailia.dll, libailia.so 等)をアンロードするまで有効です。
	*   AILIANetwork のインスタンスがある場合は ailiaGetErrorDetail() でエラーの詳細を取得できます。
	*
	* \~english
	* @brief Returns the string describing given status code.
	* @return
	*   String describing given status code.
	*   Retuned string is valid until the library of ailia (ailia.dll, libailia.so, etc) is unloaded.
	* @details
	*   The return value does not have to be released.
	*   If an instance of AILIANetwork is exist, ailiaGetErrorDetail() can be used to get the detail of the error.
	*/
	const char * AILIA_API ailiaGetStatusString(int status_code);

	/**
	* \~japanese
	* @brief エラーの詳細を返します
	* @return
	*   エラー詳細
	* @details
	*   返値は解放する必要はありません。
	*   文字列の有効期間は次にailiaのAPIを呼ぶまでです。
	*   モデルが暗号化されている場合は空文字を返します。
	*
	* \~english
	* @brief Returns the details of errors.
	* @return
	*   Error details
	* @details
	*   The return value does not have to be released.
	*   The string is valid until the next ailia API function is called.
	*   If model is encrypted, this function returns empty string.
	*/
	const char * AILIA_API ailiaGetErrorDetail(struct AILIANetwork *net);

	/**
	* \~japanese
	* @brief ライブラリバージョンを取得します
	* @return
	*   バージョン番号
	* @details
	*   返値は解放する必要はありません。
	*
	* \~english
	* @brief Get the version of the library.
	* @return
	*   Version number
	* @details
	*   The return value does not have to be released.
	*/
	const char* AILIA_API ailiaGetVersion(void);

	#ifdef UNICODE
		#define ailiaOpenStreamFile                 ailiaOpenStreamFileW
		#define ailiaOpenWeightFile                 ailiaOpenWeightFileW
		#define ailiaSetTemporaryCachePath          ailiaSetTemporaryCachePathW
	#else
		#define ailiaOpenStreamFile                 ailiaOpenStreamFileA
		#define ailiaOpenWeightFile                 ailiaOpenWeightFileA
		#define ailiaSetTemporaryCachePath          ailiaSetTemporaryCachePathA
	#endif

#ifdef __cplusplus
}
#endif
#endif /* !defined(INCLUDED_AILIA) */
