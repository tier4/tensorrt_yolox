/**
* \~japanese
* @file
* @brief AILIA 特徴抽出ライブラリ
* @copyright AXELL CORPORATION, ax Inc.
* @date 2021/07/28
*
* \~english
* @file
* @brief AILIA feature extraction library
* @copyright AXELL CORPORATION, ax Inc.
* @date July 28, 2021
*/
#if       !defined(INCLUDED_AILIA_FEATURE_EXTRACTOR)
#define            INCLUDED_AILIA_FEATURE_EXTRACTOR

/* コアライブラリ */

#include "ailia.h"
#include "ailia_format.h"

/* 呼び出し規約 */

#ifdef __cplusplus
extern "C" {
#endif

	/****************************************************************
	* 特徴抽出オブジェクトのインスタンス
	**/

	struct AILIAFeatureExtractor;

	/****************************************************************
	* 距離設定
	**/

	/**
	* \~japanese
	* @def AILIA_FEATURE_EXTRACTOR_DISTANCE_L2NORM
	* @brief L2ノルム
	*
	* \~english
	* @def AILIA_FEATURE_EXTRACTOR_DISTANCE_L2NORM
	* @brief L2 norm
	*/
	#define AILIA_FEATURE_EXTRACTOR_DISTANCE_L2NORM (0)

	/**
	* \~japanese
	* @brief 特徴抽出オブジェクトを作成します。
	* @param fextractor 特徴抽出オブジェクトポインタ
	* @param net        ネットワークオブジェクトポインタ
	* @param format     ネットワークの画像フォーマット (AILIA_NETWORK_IMAGE_FORMAT_*)
	* @param channel    ネットワークの画像チャンネル (AILIA_NETWORK_IMAGE_CHANNEL_*)
	* @param range      ネットワークの画像レンジ (AILIA_NETWORK_IMAGE_RANGE_*)
	* @param layer_name 特徴に対応したレイヤーの名称 (VGG16の場合はfc1, NULLで最終レイヤー)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Creates a feature extraction instance.
	* @param fextractor A feature extraction instance pointer
	* @param net        A network instance pointer
	* @param format     The network image format (AILIA_NETWORK_IMAGE_FORMAT_*)
	* @param channel    The network image channel (AILIA_NETWORK_IMAGE_CHANNEL_*)
	* @param range      The network image range (AILIA_NETWORK_IMAGE_RANGE_*)
	* @param layer_name The name of the layer corresponding to the feature (fc1 for VGG16 and NULL for the last layer)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaCreateFeatureExtractor(struct AILIAFeatureExtractor **fextractor, struct AILIANetwork *net, unsigned int format, unsigned int channel, unsigned int range, const char *layer_name);

	/**
	* \~japanese
	* @brief 特徴抽出オブジェクトを破棄します。
	* @param fextractor 特徴抽出オブジェクトポインタ
	*
	* \~english
	* @brief It destroys the feature extraction instance.
	* @param fextractor A feature extraction instance pointer
	*/
	void AILIA_API ailiaDestroyFeatureExtractor(struct AILIAFeatureExtractor *fextractor);

	/**
	* \~japanese
	* @brief 特徴の抽出を行います。
	* @param fextractor                  特徴抽出オブジェクトポインタ
	* @param dst                         特徴の格納先ポインタ(numeric型)
	* @param dst_size                    dstのサイズ(byte)
	* @param src                         画像データ(32bpp)
	* @param src_stride                  1ラインのバイト数
	* @param src_width                   画像幅
	* @param src_height                  画像高さ
	* @param src_format                  画像フォーマット (AILIA_IMAGE_FORMAT_*)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Performs feature extraction.
	* @param fextractor                  A feature extraction instance pointer
	* @param dst                         A pointer to the storage location of the feature (numeric type)
	* @param dst_size                    The size of the dst (bytes)
	* @param src                         Image data (32 bpp)
	* @param src_stride                  The number of bytes in 1 line
	* @param src_width                   Image width
	* @param src_height                  Image height
	* @param src_format                  Image format (AILIA_IMAGE_FORMAT_*)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaFeatureExtractorCompute(struct AILIAFeatureExtractor *fextractor, void *dst, unsigned int dst_size, const void *src, unsigned int src_stride, unsigned int src_width, unsigned int src_height, unsigned int src_format);

	/**
	* \~japanese
	* @brief 特徴間の距離を計算します。
	* @param fextractor                  特徴抽出オブジェクトポインタ
	* @param distance                    特徴間距離
	* @param distance_type               特徴間距離の種別
	* @param feature1                    一方の特徴の格納先ポインタ(numeric型)
	* @param feature1_size               dstのサイズ(byte)
	* @param feature2                    他方の特徴の格納先ポインタ(numeric型)
	* @param feature2_size               dstのサイズ(byte)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Computes distances in feature space.
	* @param fextractor                  A feature extraction instance pointer
	* @param distance                    A distance in feature space
	* @param distance_type               The type of the distance in feature space
	* @param feature1                    A pointer to the storage location of one feature (numeric type)
	* @param feature1_size               The size of the feature1 (bytes)
	* @param feature2                    A pointer to the storage location of the other feature (numeric type)
	* @param feature2_size               The size of the feature2 (bytes)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaFeatureExtractorMatch(struct AILIAFeatureExtractor *fextractor, float *distance, unsigned int distace_type, const void *feature1, unsigned int feature1_size, const void *feature2, unsigned int feature2_size);

#ifdef __cplusplus
}
#endif
#endif /* !defined(INCLUDED_AILIA_FEATURE_EXTRACTOR) */
