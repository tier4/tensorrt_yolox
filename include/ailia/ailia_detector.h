/**
* \~japanese
* @file
* @brief AILIA 物体検出ライブラリ
* @copyright AXELL CORPORATION, ax Inc.
* @date 2021/07/28
*
* \~english
* @file
* @brief AILIA object detection library
* @copyright AXELL CORPORATION, ax Inc.
* @date July 28, 2021
*/
#if       !defined(INCLUDED_AILIA_DETECTOR)
#define            INCLUDED_AILIA_DETECTOR

/* コアライブラリ */

#include "ailia.h"
#include "ailia_format.h"

/* 呼び出し規約 */

#ifdef __cplusplus
extern "C" {
#endif

	/****************************************************************
	* 検出オブジェクトのインスタンス
	**/

	struct AILIADetector;

	/****************************************************************
	* 物体情報
	**/
	#define AILIA_DETECTOR_OBJECT_VERSION (1)

	typedef struct _AILIADetectorObject {
		/**
		* \~japanese
		* オブジェクトカテゴリ番号(0～category_count-1)
		*
		* \~english
		* Object category number (0 to category_count-1)
		*/
		unsigned int category;
		/**
		* \~japanese
		* 推定確率(0～1)
		*
		* \~english
		* Estimated probability (0 to 1)
		*/
		float prob;
		/**
		* \~japanese
		* 左上X位置(1で画像幅)
		*
		* \~english
		* X position at the top left (1 for the image width)
		*/
		float x;
		/**
		* \~japanese
		* 左上Y位置(1で画像高さ)
		*
		* \~english
		* Y position at the top left (1 for the image height)
		*/
		float y;
		/**
		* \~japanese
		* 幅(1で画像横幅、負数は取らない)
		*
		* \~english
		* Width (1 for the width of the image, negative numbers not allowed)
		*/
		float w;
		/**
		* \~japanese
		* 高さ(1で画像高さ、負数は取らない)
		*
		* \~english
		* Height (1 for the height of the image, negative numbers not allowed)
		*/
		float h;
	}AILIADetectorObject;

	#define AILIA_DETECTOR_ALGORITHM_YOLOV1 (0) //YOLOV1
	#define AILIA_DETECTOR_ALGORITHM_YOLOV2 (1) //YOLOV2
	#define AILIA_DETECTOR_ALGORITHM_YOLOV3 (2) //YOLOV3
	#define AILIA_DETECTOR_ALGORITHM_YOLOV4 (3) //YOLOV4
	#define AILIA_DETECTOR_ALGORITHM_YOLOX  (4) //YOLOX
	#define AILIA_DETECTOR_ALGORITHM_SSD    (8) //SSD(Single Shot multibox Detector)

	#define AILIA_DETECTOR_FLAG_NORMAL      (0) //オプションなし

	/****************************************************************
	* 物体検出API
	**/

	/**
	* \~japanese
	* @brief 検出オブジェクトを作成します。
	* @param detector       検出オブジェクトポインタ
	* @param net            ネットワークオブジェクトポインタ
	* @param format         ネットワークの画像フォーマット (AILIA_NETWORK_IMAGE_FORMAT_*)
	* @param channel        ネットワークの画像チャンネル (AILIA_NETWORK_IMAGE_CHANNEL_*)
	* @param range          ネットワークの画像レンジ (AILIA_NETWORK_IMAGE_RANGE_*)
	* @param algorithm      検出アルゴリズム(AILIA_DETECTOR_ALGORITHM_*)
	* @param category_count 検出カテゴリ数(VOCの場合は20、COCOの場合は80、などを指定)
	* @param flags          追加オプションフラグ(AILIA_DETECTOR_FLAG_*)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Creates a detector instance.
	* @param detector       A detector instance pointer
	* @param net            The network instance pointer
	* @param format         The network image format (AILIA_NETWORK_IMAGE_FORMAT_*)
	* @param channel        The network image channel (AILIA_NETWORK_IMAGE_CHANNEL_*)
	* @param range          The network image range (AILIA_NETWORK_IMAGE_RANGE_*)
	* @param algorithm      Detection algorithm(AILIA_DETECTOR_ALGORITHM_*)
	* @param category_count The number of detection categories (specify 20 for VOC or 80 for COCO, etc.)
	* @param flags          Additional option(AILIA_DETECTOR_FLAG_*)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaCreateDetector(struct AILIADetector **detector,struct AILIANetwork *net, unsigned int format, unsigned int channel, unsigned int range, unsigned int algorithm, unsigned int category_count, unsigned int flags);

	/**
	* \~japanese
	* @brief 検出オブジェクトを破棄します。
	* @param detector 検出オブジェクトポインタ
	*
	* \~english
	* @brief Destroys the detector instance.
	* @param detector A detector instance pointer
	*/
	void AILIA_API ailiaDestroyDetector(struct AILIADetector *detector);

	/**
	* \~japanese
	* @brief 物体検出を行います。
	* @param detector                    検出オブジェクトポインタ
	* @param src                         画像データ(32bpp)
	* @param src_stride                  1ラインのバイト数
	* @param src_width                   画像幅
	* @param src_height                  画像高さ
	* @param src_format                  画像フォーマット (AILIA_IMAGE_FORMAT_*)
	* @param threshold                   検出しきい値(0.1f等)(小さいほど検出されやすくなり、検出数増加)
	* @param iou                         重複除外しきい値(0.45f等)(小さいほど重複を許容せず検出数減少)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Performs object detection.
	* @param detector                    A detector instance pointer
	* @param src                         Image data (32 bpp)
	* @param src_stride                  The number of bytes in 1 line
	* @param src_width                   Image width
	* @param src_height                  Image height
	* @param src_format                  Image format (AILIA_IMAGE_FORMAT_*)
	* @param threshold                   The detection threshold (for example, 0.1f) (The smaller it is, the easier the detection will be and the more detected objects found.)
	* @param iou                         Iou threshold (for example, 0.45f) (The smaller it is, the fewer detected objects found, as duplication is not allowed.)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaDetectorCompute(struct AILIADetector *detector, const void *src, unsigned int src_stride, unsigned int src_width, unsigned int src_height, unsigned int src_format, float threshold, float iou);

	/**
	* \~japanese
	* @brief 検出結果の数を取得します。
	* @param detector   検出オブジェクトポインタ
	* @param obj_count  オブジェクト数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the number of detection results.
	* @param detector   A detector instance pointer
	* @param obj_count  The number of objects
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaDetectorGetObjectCount(struct AILIADetector *detector, unsigned int *obj_count);

	/**
	* \~japanese
	* @brief 検出結果を取得します。
	* @param detector   検出オブジェクトポインタ
	* @param obj        オブジェクト情報
	* @param obj_idx    オブジェクトインデックス
	* @param version    AILIA_DETECTOR_OBJECT_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*    ailiaDetectorCompute() を一度も実行していない場合は \ref AILIA_STATUS_INVALID_STATE が返ります。
	*   検出結果は推定確率順でソートされます。
	*
	* \~english
	* @brief Gets the detection results.
	* @param detector   A detector instance pointer
	* @param obj        Object information
	* @param obj_idx    Object index
	* @param version    AILIA_DETECTOR_OBJECT_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   If  ailiaDetectorCompute()  is not run at all, the function returns  \ref AILIA_STATUS_INVALID_STATE .
	*   The detection results are sorted in the order of estimated probability.
	*/
	int AILIA_API ailiaDetectorGetObject(struct AILIADetector *detector, AILIADetectorObject* obj, unsigned int obj_idx, unsigned int version);

	/**
	* \~japanese
	* @brief YoloV2などのためにアンカーズ (anchorsまたはbiases) の情報を設定します。
	* @param detector       検出オブジェクトポインタ
	* @param anchors        アンカーズの寸法 (検出ボックスの形状、高さと幅)
	* @param anchors_count  アンカーズの数 (anchorsの配列サイズの半分)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   YoloV2などは学習時に決定された複数の検出ボックスを使用して物体検出を行います。このAPIで学習時に決定された検出ボックスの形状を設定することで、正しい推論を行います。
	*   anchorsには{x,y,x,y...}の形式で格納します。
	*   anchors_countが5の場合、anchorsは10次元の配列になります。
	*
	* \~english
	* @brief Sets the anchor information (anchors or biases) for YoloV2 or other systems.
	* @param detector       A detector instance pointer
	* @param anchors        The anchor dimensions (the shape, height and width of the detection box)
	* @param anchors_count  The number of anchors (half of the anchors array size)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   YoloV2 and other systems perform object detection with multiple detection boxes determined during training. By using this API function to set the shape of the detection box determined during training, correct inferences can be made.
	*   The {x, y, x, y ...} format is used for anchor storage.
	*   If anchors_count has a value of 5, then anchors is a 10-dimensional array.
	*/
	int AILIA_API ailiaDetectorSetAnchors(struct AILIADetector *detector, float *anchors, unsigned int anchors_count);

	/**
	* \~japanese
	* @brief YoloV3またはYoloXでのモデルへの入力画像サイズを指定します。
	* @param detector       検出オブジェクトポインタ
	* @param input_width    モデルの入力画像幅
	* @param input_height   モデルの入力画像高さ
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   YoloV3では単一のモデルが任意の入力解像度に対応します。(32 の倍数制限あり)
	*   YoloXでは単一のモデルが任意の入力解像度に対応します。
	*   計算量の削減等でモデルへの入力画像サイズを指定する場合この API を実行してください。
	*    ailiaCreateDetector() () と  ailiaDetectorCompute() () の間に実行する必要があります。
	*   この API を実行しない場合、デフォルトの 416x416 を利用します。
	*   YOLOv3またはYOLOX 以外で実行した場合、 \ref AILIA_STATUS_INVALID_STATE  を返します。
	*
	* \~english
	* @brief Sets the size of the input image for YoloV3 or YoloX model.
	* @param detector       A detector instance pointer
	* @param input_width    Width of the model's input image
	* @param input_height   Height of the model's input image
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   The same YoloV3 model can be used for any input image size that is a multiple of 32.
	*   The same YoloX model can be used for any input image size.
	*   You can use this API if you want to choose the input image size, for example to reduce the calculation complexity.
	*   It must be called between  ailiaCreateDetector() () and  ailiaDetectorCompute() ().
	*   If this API is not used, a default size of 416x416 is assumed.
	*   If used with some model other than YoloV3 or YoloX, it will return the error status  \ref AILIA_STATUS_INVALID_STATE .
	*/
	int AILIA_API ailiaDetectorSetInputShape(struct AILIADetector *detector, unsigned int input_width, unsigned int input_height);

#ifdef __cplusplus
}
#endif
#endif /* !defined(INCLUDED_AILIA_DETECTOR) */
