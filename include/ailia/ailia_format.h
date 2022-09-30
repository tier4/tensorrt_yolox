/**
* \~japanese
* @file
* @brief AILIA フォーマット定義・変換
* @copyright AXELL CORPORATION, ax Inc.
* @date 2021/07/28
*
* \~english
* @file
* @brief AILIA format definition and conversion
* @copyright AXELL CORPORATION, ax Inc.
* @date July 28, 2021
*/
#if       !defined(INCLUDED_AILIA_FORMAT)
#define            INCLUDED_AILIA_FORMAT

/* 呼び出し規約 */

#if defined(_M_JS)
#include <emscripten.h>
#define AILIA_API EMSCRIPTEN_KEEPALIVE
#elif defined(_WIN64) || defined(_M_X64) || defined(__amd64__) || defined(__x86_64__) || defined(__APPLE__) || defined(__ANDROID__) || defined(ANDROID) || defined(__linux__) || defined(NN_NINTENDO_SDK)
#define AILIA_API
#else
#define AILIA_API __stdcall
#endif

#ifdef __cplusplus
extern "C" {
#endif

	/****************************************************************
	* 入力画像フォーマット
	**/

	/**
	* \~japanese
	* RGBA順
	*
	* \~english
	* RGBA order
	*/
	#define AILIA_IMAGE_FORMAT_RGBA        (0x00)
	/**
	* \~japanese
	* BGRA順
	*
	* \~english
	* BGRA order
	*/
	#define AILIA_IMAGE_FORMAT_BGRA        (0x01)
	/**
	* \~japanese
	* RGB順
	*
	* \~english
	* RGB order
	*/
	#define AILIA_IMAGE_FORMAT_RGB         (0x02)
	/**
	* \~japanese
	* BGR順
	*
	* \~english
	* BGR order
	*/
	#define AILIA_IMAGE_FORMAT_BGR         (0x03)

	/**
	* \~japanese
	* RGBA順(Bottom to Top)
	*
	* \~english
	* RGBA order (Bottom to top)
	*/
	#define AILIA_IMAGE_FORMAT_RGBA_B2T    (0x10)
	/**
	* \~japanese
	* BGRA順(Bottom to Top)
	*
	* \~english
	* BGRA order (Bottom to top)
	*/
	#define AILIA_IMAGE_FORMAT_BGRA_B2T    (0x11)

	/****************************************************************
	* ネットワーク画像フォーマット
	**/

	/**
	* \~japanese
	* BGR順
	*
	* \~english
	* BGR order
	*/
	#define AILIA_NETWORK_IMAGE_FORMAT_BGR               (0)
	/**
	* \~japanese
	* RGB順
	*
	* \~english
	* RGB order
	*/
	#define AILIA_NETWORK_IMAGE_FORMAT_RGB               (1)
	/**
	* \~japanese
	* Gray Scale (1ch)
	*
	* \~english
	* Gray Scale (1ch)
	*/
	#define AILIA_NETWORK_IMAGE_FORMAT_GRAY              (2)
	/**
	* \~japanese
	* ヒストグラム平坦化 Gray Scale (1ch)
	*
	* \~english
	* Equalized Gray Scale (1ch)
	*/
	#define AILIA_NETWORK_IMAGE_FORMAT_GRAY_EQUALIZE     (3)

	/**
	* \~japanese
	* DCYX順
	*
	* \~english
	* DCYX order
	*/
	#define AILIA_NETWORK_IMAGE_CHANNEL_FIRST            (0)
	/**
	* \~japanese
	* DYXC順
	*
	* \~english
	* DYXC order
	*/
	#define AILIA_NETWORK_IMAGE_CHANNEL_LAST             (1)

	/**
	* \~japanese
	* 0 ～ 255
	*
	* \~english
	* 0 to 255
	*/
	#define AILIA_NETWORK_IMAGE_RANGE_UNSIGNED_INT8      (0)
	/**
	* \~japanese
	* -128 ～ 127
	*
	* \~english
	* -128 to 127
	*/
	#define AILIA_NETWORK_IMAGE_RANGE_SIGNED_INT8        (1)
	/**
	* \~japanese
	* 0.0 ～ 1.0
	*
	* \~english
	* 0.0 to 1.0
	*/
	#define AILIA_NETWORK_IMAGE_RANGE_UNSIGNED_FP32      (2)
	/**
	* \~japanese
	* -1.0 ～ 1.0
	*
	* \~english
	* -1.0 to 1.0
	*/
	#define AILIA_NETWORK_IMAGE_RANGE_SIGNED_FP32        (3)
	/**
	* \~japanese
	* ImageNet mean&std normalize
	*
	* \~english
	* ImageNet mean&std normalize
	*/
	#define AILIA_NETWORK_IMAGE_RANGE_IMAGENET           (4)


	/**
	* \~japanese
	* @brief 画像のフォーマットを変換します。
	* @param dst                  変換後画像の格納先(numeric型、sizeof(float) * dst_width * dst_height * チャンネル数(解説参照)以上のサイズを確保すること)
	* @param dst_width            変換後画像の横幅
	* @param dst_height           変換後画像の高さ
	* @param dst_format           変換後画像の形式 (AILIA_NETWORK_IMAGE_FORMAT_*)
	* @param dst_channel          変換後画像のチャンネル順 (AILIA_NETWORK_IMAGE_CHANNEL_*)
	* @param dst_range            変換後画像のレンジ (AILIA_NETWORK_IMAGE_RANGE_*)
	* @param src                  変換元画像の格納先(32bpp)
	* @param src_stride           変換元画像のラインバイト数
	* @param src_width            変換元画像の横幅
	* @param src_height           変換元画像の高さ
	* @param src_format           変換元画像の形式 (AILIA_IMAGE_FORMAT_*)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   画像フォーマットを変更します。dst_formatが \ref AILIA_NETWORK_IMAGE_FORMAT_BGR もしくはAILIA_NETWORK_IMAGE_FORMAT_RGB
	*   の場合、チャンネル数は3,  \ref AILIA_NETWORK_IMAGE_FORMAT_GRAY の場合チャンネル数は1となります。
	*
	* \~english
	* @brief Converts image formats.
	* @param dst                  The storage location of the image after conversion (numeric type; a size of sizeof(float) * dst_width * dst_height * num_of_channel(See Description) or more must be allocated.)
	* @param dst_width            The width of the image after conversion
	* @param dst_height           The height of the image after conversion
	* @param dst_format           The format of the image after conversion (AILIA_NETWORK_IMAGE_FORMAT_*)
	* @param dst_channel          The channel order of the image after conversion (AILIA_NETWORK_IMAGE_CHANNEL_*)
	* @param dst_range            The range of the image after conversion (AILIA_NETWORK_IMAGE_RANGE_*)
	* @param src                  The storage location of the source image (32 bpp)
	* @param src_stride           The line byte number of the source image
	* @param src_width            The width of the source image
	* @param src_height           The height of the source image
	* @param src_format           The format of the source image (AILIA_IMAGE_FORMAT_*)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function converts image formats. If dst_format is set to  \ref AILIA_NETWORK_IMAGE_FORMAT_BGR  or  \ref AILIA_NETWORK_IMAGE_FORMAT_RGB ,
	*   the number of channels is 3, otherwise if set to  \ref AILIA_NETWORK_IMAGE_FORMAT_GRAY , the number of channels is 1.
	*/
	int AILIA_API ailiaFormatConvert(void *dst, unsigned int dst_width, unsigned int dst_height, unsigned int dst_format, unsigned int dst_channel, unsigned int dst_range, const void *src, int src_stride, unsigned int src_width, unsigned int src_height, unsigned int src_format);

#ifdef __cplusplus
}
#endif
#endif /* !defined(INCLUDED_AILIA_FORMAT) */
