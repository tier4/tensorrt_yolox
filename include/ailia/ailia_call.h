/**
 * \~japanese
 * @file ailia_call.h
 * @brief ユーザ定義コールバック
 * @copyright AXELL CORPORATION, ax Inc.
 * @date 2021/07/28
 * @details
 * ファイルアクセスコールバック関数は、1ネットワークオブジェクトにつき、
 * 1スレッドから呼び出されます。
 * 複数のオブジェクトに対して、共通のコールバック関数を与える場合は、
 * コールバック関数はスレッドセーフである必要があります。
 * また、コールバック関数からは例外を投げずに、
 * AILIA_USER_API_FAILEDでエラーを通知してください。
 *
 * \~english
 * @file ailia_call.h
 * @brief user-defined callback
 * @copyright AXELL CORPORATION, ax Inc.
 * @date 2021/07/28
 * @details
 * The file access callback function is called by a single thread
 * per network instance.
 * If a common callback function is given to multiple instances,
 * the callback function must be thread-safe.
 * Also, do not throw an exception from the callback function,
 * instead use AILIA_USER_API_FAILED to make error notifications.
 */

#if       !defined(INCLUDED_AILIA_CALL)
#define            INCLUDED_AILIA_CALL

/* 呼び出し規約 */

#if defined(_WIN32) && !defined(_WIN64)
	#define AILIA_USER_API __stdcall
#else
	#define AILIA_USER_API
#endif

#ifdef __EMSCRIPTEN__
	#define AILIA_FSIZE_RETURN_TYPE int
#else
	#define AILIA_FSIZE_RETURN_TYPE long long
#endif

/* ユーザ定義コールバックの返値 */
/**
* \~japanese
* @def AILIA_USER_API_SUCCESS
* @brief 成功
*
* \~english
* @def AILIA_USER_API_SUCCESS
* @brief Successful
*/
#define AILIA_USER_API_SUCCESS ( 0)
/**
* \~japanese
* @def AILIA_USER_API_FAILED
* @brief 失敗
*
* \~english
* @def AILIA_USER_API_FAILED
* @brief Failed
*/
#define AILIA_USER_API_FAILED  (-1)

/*
* ファイルアクセスコールバック関数は、1ネットワークオブジェクトにつき、
* 1スレッドから呼び出されます。
* 複数のオブジェクトに対して、共通のコールバック関数を与える場合は、
* コールバック関数はスレッドセーフである必要があります。
* また、コールバック関数からは例外を投げずに、
* AILIA_USER_API_FAILEDでエラーを通知してください。
*/


#ifdef __cplusplus
extern "C" {
#endif

	/**
	* \~japanese
	* @brief ファイルを開きます
	* @param const void *   ailiaOpenStreamEx() もしくは ailiaOpenWeightEx() に与えたfopen_args
	* @return
	*   成功した場合、ユーザ定義ファイルポインタを返す。
	*   失敗した場合、NULLを返す。
	*
	* \~english
	* @brief Opens a file.
	* @param const void *  fopen_args given to  ailiaOpenStreamEx()  or ailiaOpenWeightEx
	* @return
	*   This function returns a user-defined file pointer if successful.
	*   It returns NULL if it fails.
	*/
	typedef void* (AILIA_USER_API *AILIA_USER_API_FOPEN) (const void *);

	/**
	* \~japanese
	* @brief ファイルをシークします
	* @param void *                     ユーザ定義ファイルポインタ
	* @param  \ref AILIA_FSIZE_RETURN_TYPE     ファイル先頭からのオフセットバイト
	* @return
	*   成功した場合、 \ref AILIA_USER_API_SUCCESS を返す。
	*   失敗した場合、 \ref AILIA_USER_API_FAILED を返す。
	*
	* \~english
	* @brief It seeks the file specified.
	* @param void *                     A user-defined file pointer
	* @param  \ref AILIA_FSIZE_RETURN_TYPE     Offset in bytes from the beginning of the file
	* @return
	*   This function returns  \ref AILIA_USER_API_SUCCESS  if successful.
	*   It returns  \ref AILIA_USER_API_FAILED  if it fails.
	*/
	typedef int (AILIA_USER_API *AILIA_USER_API_FSEEK) (void*, AILIA_FSIZE_RETURN_TYPE);

	/**
	* \~japanese
	* @brief ファイルの現在位置を取得します
	* @param void * ユーザ定義ファイルポインタ
	* @return
	*   成功した場合、ファイルポインタの位置をバイト単位で返す。
	*   失敗した場合、-1を返す。
	*
	* \~english
	* @brief Gets the current position in the file.
	* @param void * A user-defined file pointer
	* @return
	*   This function returns the position, in bytes, the file pointer points to if successful.
	*   It returns -1 if it fails.
	*/
	typedef AILIA_FSIZE_RETURN_TYPE (AILIA_USER_API *AILIA_USER_API_FTELL) (void*);

	/**
	* \~japanese
	* @brief ファイルのサイズを取得します
	* @param void * ユーザ定義ファイルポインタ
	* @return
	*   成功した場合、ファイルのサイズをバイト単位で返す。
	*   失敗した場合、-1を返す。
	*
	* \~english
	* @brief Gets the size of the file.
	* @param void * A user-defined file pointer
	* @return
	*   This function returns the size of the file in bytes if successful.
	*   It returns -1 if it fails.
	*/
	typedef AILIA_FSIZE_RETURN_TYPE (AILIA_USER_API *AILIA_USER_API_FSIZE) (void*);

	/**
	* \~japanese
	* @brief ファイルからデータを読み込みます
	* @param void *                     読み込みデータ格納先のポインタ
	* @param  \ref AILIA_FSIZE_RETURN_TYPE     読み込みデータのバイト長さ
	* @param void *                     ユーザ定義ファイルポインタ
	* @return
	*   成功した場合、 \ref AILIA_USER_API_SUCCESS を返す。
	*   失敗した場合、 \ref AILIA_USER_API_FAILED を返す。
	*   標準APIとは異なり、返値はAILIA_USER_API_*になりますのでご注意ください。
	*
	* \~english
	* @brief Reads data from the file.
	* @param void *                     A pointer to the storage location of the data to be read
	* @param  \ref AILIA_FSIZE_RETURN_TYPE     The length in bytes of the data to be read
	* @param void *                     A user-defined file pointer
	* @return
	*   This function returns  \ref AILIA_USER_API_SUCCESS  if successful.
	*   It returns  \ref AILIA_USER_API_FAILED  if it fails.
	*   Note that unlike the standard API, the return value will be AILIA_USER_API_*.
	*/
	typedef int (AILIA_USER_API *AILIA_USER_API_FREAD) (void *, AILIA_FSIZE_RETURN_TYPE, void*);

	/**
	* \~japanese
	* @brief ファイルを閉じます
	* @param void *  ユーザ定義ファイルポインタ
	* @return
	*   成功した場合、 \ref AILIA_USER_API_SUCCESS を返す。
	*   失敗した場合、 \ref AILIA_USER_API_FAILED を返す。
	*
	* \~english
	* @brief Closes the file.
	* @param void *  A user-defined file pointer
	* @return
	*   This function returns  \ref AILIA_USER_API_SUCCESS  if successful.
	*   It returns  \ref AILIA_USER_API_FAILED  if it fails.
	*/
	typedef int (AILIA_USER_API *AILIA_USER_API_FCLOSE) (void*);

	/**
	* \~japanese
	* @def AILIA_FILE_CALLBACK_VERSION
	* @brief 構造体バージョン
	*
	* \~english
	* @def AILIA_FILE_CALLBACK_VERSION
	* @brief Struct version
	*/
	#define AILIA_FILE_CALLBACK_VERSION (1)

	/* ファイルアクセスコールバック関数構造体 */
	typedef struct _ailiaFileCallback {
		AILIA_USER_API_FOPEN  fopen;     /* ユーザ定義fopen関数 */
		AILIA_USER_API_FSEEK  fseek;     /* ユーザ定義fseek関数 */
		AILIA_USER_API_FTELL  ftell;     /* ユーザ定義ftell関数 */
		AILIA_USER_API_FREAD  fread;     /* ユーザ定義fread関数 */
		AILIA_USER_API_FSIZE  fsize;     /* ユーザ定義fsize関数 */
		AILIA_USER_API_FCLOSE fclose;    /* ユーザ定義fclose関数 */
	} ailiaFileCallback;



#ifdef __cplusplus
}
#endif
#endif /* !defined(INCLUDED_AILIA_CALL) */
