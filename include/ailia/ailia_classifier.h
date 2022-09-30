/**
 * \~japanese
 * @file ailia_classifier.h
 * @brief 物体識別ライブラリ
 * @copyright AXELL CORPORATION, ax Inc.
 * @date 2021/07/28
 *
 * \~english
 * @file ailia_classifier.h
 * @brief object classification library
 * @copyright AXELL CORPORATION, ax Inc.
 * @date 2021/07/28
 */

#if       !defined(INCLUDED_AILIA_CLASSIFIER)
#define            INCLUDED_AILIA_CLASSIFIER

/* コアライブラリ */

#include "ailia.h"
#include "ailia_format.h"

/* 呼び出し規約 */

#ifdef __cplusplus
extern "C" {
#endif

	/****************************************************************
	* 識別オブジェクトのインスタンス
	**/

	struct AILIAClassifier;

	/****************************************************************
	* 識別情報
	**/

	#define AILIA_CLASSIFIER_CLASS_VERSION (1)

	typedef struct _AILIAClassifierClass {
		/**
		* \~japanese
		* 識別カテゴリ番号
		*
		* \~english
		* Classification category number
		*/
		int category;
		/**
		* \~japanese
		* 推定確率(0～1)
		*
		* \~english
		* Estimated probability (0 to 1)
		*/
		float prob;
	}AILIAClassifierClass;

	/**
	* \~japanese
	* @brief 識別オブジェクトを作成します。
	* @param classifier 識別オブジェクトポインタへのポインタ
	* @param net        ネットワークオブジェクトポインタ
	* @param format     ネットワークの画像フォーマット （AILIA_NETWORK_IMAGE_FORMAT_*）
	* @param channel    ネットワークの画像チャンネル (AILIA_NETWORK_IMAGE_CHANNEL_*)
	* @param range      ネットワークの画像レンジ （AILIA_NETWORK_IMAGE_RANGE_*）
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Creates a classifier instance.
	* @param classifier A pointer to a classifier instance pointer
	* @param net        A network instance pointer
	* @param format     The network image format (AILIA_NETWORK_IMAGE_FORMAT_*)
	* @param channel    The network image channel (AILIA_NETWORK_IMAGE_CHANNEL_*)
	* @param range      The network image range (AILIA_NETWORK_IMAGE_RANGE_*)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaCreateClassifier(struct AILIAClassifier **classifier, struct AILIANetwork *net, unsigned int format, unsigned int channel, unsigned int range);

	/**
	* \~japanese
	* @brief 識別オブジェクトを破棄します。
	* @param classifier 識別オブジェクトポインタ
	*
	* \~english
	* @brief Destroys the classifier instance.
	* @param classifier A classifier instance pointer
	*/
	void AILIA_API ailiaDestroyClassifier(struct AILIAClassifier *classifier);

	/**
	* \~japanese
	* @brief 物体識別を行います。
	* @param classifier                  識別オブジェクトポインタ
	* @param src                         画像データ(32bpp)
	* @param src_stride                  1ラインのバイト数
	* @param src_width                   画像幅
	* @param src_height                  画像高さ
	* @param src_format                  画像のフォーマット (AILIA_IMAGE_FORMAT_*)
	* @param max_class_count             識別結果の数の最大
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Performs object classification.
	* @param classifier                  A classifier instance pointer
	* @param src                         Image data (32 bpp)
	* @param src_stride                  The number of bytes in 1 line
	* @param src_width                   Image width
	* @param src_height                  Image height
	* @param src_format                  Image format (AILIA_IMAGE_FORMAT_*)
	* @param max_class_count             The maximum number of classification results
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaClassifierCompute(struct AILIAClassifier *classifier,const void *src, unsigned int src_stride, unsigned int src_width, unsigned int src_height, unsigned int src_format, unsigned int max_class_count);

	/**
	* \~japanese
	* @brief 識別結果の数を取得します。
	* @param classifier 識別オブジェクトポインタ
	* @param cls_count  クラス数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the number of classification results.
	* @param classifier A classifier instance pointer
	* @param cls_count  The number of classes
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaClassifierGetClassCount(struct AILIAClassifier *classifier, unsigned int *cls_count);

	/**
	* \~japanese
	* @brief 識別結果を取得します。
	* @param classifier 識別オブジェクトポインタ
	* @param cls        クラス情報
	* @param cls_idx    クラスインデックス
	* @param version    \ref AILIA_CLASSIFIER_CLASS_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*    ailiaClassifierCompute() を一度も実行していない場合は \ref AILIA_STATUS_INVALID_STATE が返ります。
	*   識別結果は推定確率順でソートされます。
	*
	* \~english
	* @brief Gets the classification results.
	* @param classifier A classifier instance pointer
	* @param cls        Class information
	* @param cls_idx    Class index
	* @param version    \ref AILIA_CLASSIFIER_CLASS_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   If  ailiaClassifierCompute()  is not run at all, the function returns  \ref AILIA_STATUS_INVALID_STATE .
	*   The classification results are sorted in the order of estimated probability.
	*/
	int AILIA_API ailiaClassifierGetClass(struct AILIAClassifier *classifier, AILIAClassifierClass* obj, unsigned int cls_idx, unsigned int version);

#ifdef __cplusplus
}
#endif
#endif /* !defined(INCLUDED_AILIA_CLASSIFIER) */
