/**
* \~japanese
* @file
* @brief AILIA 骨格検出・顔特徴点検出ライブラリ
* @copyright AXELL CORPORATION, ax Inc.
* @date 2021/07/28
*
* \~english
* @file
* @brief AILIA library for human pose estimation and human face landmarks extraction
* @copyright AXELL CORPORATION, ax Inc.
* @date July 28, 2021
*/
#if       !defined(INCLUDED_AILIA_POSE_ESTIMATOR)
#define            INCLUDED_AILIA_POSE_ESTIMATOR

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

	struct AILIAPoseEstimator;

	/****************************************************************
	* 物体情報
	**/

	/**
	* \~japanese
	* 骨格検出
	*
	* \~english
	* Human pose estimation
	*/
	#define AILIA_POSE_ESTIMATOR_ALGORITHM_ACCULUS_POSE             ( 0) 
	/**
	* \~japanese
	* 顔特徴点検出
	*
	* \~english
	* Human face landmarks extraction
	*/
	#define AILIA_POSE_ESTIMATOR_ALGORITHM_ACCULUS_FACE             ( 1) 
	/**
	* \~japanese
	* 近接上半身姿勢検出
	*
	* \~english
	* Human upper body pose estimation
	*/
	#define AILIA_POSE_ESTIMATOR_ALGORITHM_ACCULUS_UPPOSE           ( 2) 
	/**
	* \~japanese
	* 近接上半身2姿勢検出(FPGA向け)
	*
	* \~english
	* Human upper body pose estimation(FPGA)
	*/
	#define AILIA_POSE_ESTIMATOR_ALGORITHM_ACCULUS_UPPOSE_FPGA      ( 3) 
	/**
	* \~japanese
	* 手姿勢検出
	*
	* \~english
	* Human hand estimation
	*/
	#define AILIA_POSE_ESTIMATOR_ALGORITHM_ACCULUS_HAND             ( 5) 
	/**
	* \~japanese
	* 骨格検出
	*
	* \~english
	* Human pose estimation
	*/
	#define AILIA_POSE_ESTIMATOR_ALGORITHM_OPEN_POSE                (10) 
	/**
	* \~japanese
	* 骨格検出
	*
	* \~english
	* Human pose estimation
	*/
	#define AILIA_POSE_ESTIMATOR_ALGORITHM_LW_HUMAN_POSE            (11) 
	/**
	* \~japanese
	* 骨格検出
	*
	* \~english
	* Human pose estimation
	*/
	#define AILIA_POSE_ESTIMATOR_ALGORITHM_OPEN_POSE_SINGLE_SCALE   (12) 

	/* 骨格検出 関節点 定義 */
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_NOSE             (0)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_EYE_LEFT         (1)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_EYE_RIGHT        (2)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_EAR_LEFT         (3)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_EAR_RIGHT        (4)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_SHOULDER_LEFT    (5)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_SHOULDER_RIGHT   (6)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_ELBOW_LEFT       (7)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_ELBOW_RIGHT      (8)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_WRIST_LEFT       (9)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_WRIST_RIGHT      (10)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_HIP_LEFT         (11)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_HIP_RIGHT        (12)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_KNEE_LEFT        (13)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_KNEE_RIGHT       (14)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_ANKLE_LEFT       (15)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_ANKLE_RIGHT      (16)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_SHOULDER_CENTER  (17)
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_BODY_CENTER      (18)
	/**
	* \~japanese
	* 個数
	*
	* \~english
	* Count
	*/
	#define AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_CNT              (19) 

	/* 顔特徴点検出 定義 */
	/**
	* \~japanese
	* 個数
	*
	* \~english
	* Count
	*/
	#define AILIA_POSE_ESTIMATOR_FACE_KEYPOINT_CNT              (68) 

	/* 近接上半身姿勢検出 関節点 定義 */
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_NOSE               (0)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_EYE_LEFT           (1)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_EYE_RIGHT          (2)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_EAR_LEFT           (3)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_EAR_RIGHT          (4)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_SHOULDER_LEFT      (5)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_SHOULDER_RIGHT     (6)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_ELBOW_LEFT         (7)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_ELBOW_RIGHT        (8)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_WRIST_LEFT         (9)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_WRIST_RIGHT        (10)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_HIP_LEFT           (11)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_HIP_RIGHT          (12)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_SHOULDER_CENTER    (13)
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_BODY_CENTER        (14)
	/**
	* \~japanese
	* 個数
	*
	* \~english
	* Count
	*/
	#define AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_CNT                (15) 

	/* 手姿勢検出 点 定義 */
	/**
	* \~japanese
	* 個数
	*
	* \~english
	* Count
	*/
	#define AILIA_POSE_ESTIMATOR_HAND_KEYPOINT_CNT                  (21) 

	typedef struct _AILIAPoseEstimatorKeypoint {
		/**
		* \~japanese
		* 入力画像内 X座標  (0.0 , 1.0)
		*
		* \~english
		* Input image X coordinate (0.0, 1.0)
		*/
		float x;                    
		/**
		* \~japanese
		* 入力画像内 Y座標  (0.0 , 1.0)
		*
		* \~english
		* Input image Y coordinate (0.0, 1.0)
		*/
		float y;                    
		/**
		* \~japanese
		* 骨格検出のみ有効。体中心を座標0とした時に推定されるローカルZ座標。単位(スケール)は x と同じです。
		*
		* \~english
		* Valid only for human pose estimation. The local Z coordinate is estimated when the center of the body is defined as coordinate 0. The unit (scale) is the same as that for X.
		*/
		float z_local;              
		/**
		* \~japanese
		* この点の検出信頼度。値が0.0Fの場合、この点は未検出のため使用できません。
		*
		* \~english
		* The confidence of this point. If the value is 0.0F, then this point is not available as it has not been detected yet.
		*/
		float score;                
		/**
		* \~japanese
		* 通常は0です。この点が未検出で、他の点から補間可能な場合、x,yの値を補間し、interpolated=1となります。
		*
		* \~english
		* The default is 0. If this point has not been detected and can be interpolated by other points, the x and y values are then interpolated and the value of interpolated is set to 1.
		*/
		int interpolated;           
	}AILIAPoseEstimatorKeypoint;

	/**
	* \~japanese
	* 構造体フォーマットバージョン
	*
	* \~english
	* Version of the struct format
	*/
	#define AILIA_POSE_ESTIMATOR_OBJECT_POSE_VERSION (1) 
	typedef struct _AILIAPoseEstimatorObjectPose {
		/**
		* \~japanese
		* 検出した関節点。配列インデックスが関節番号に相当します。
		*
		* \~english
		* Detected body joint positions. The array index corresponding to a body joint number.
		*/
		AILIAPoseEstimatorKeypoint points[AILIA_POSE_ESTIMATOR_POSE_KEYPOINT_CNT]; 
		/**
		* \~japanese
		* このオブジェクトの検出信頼度
		*
		* \~english
		* The confidence of this object
		*/
		float total_score;          
		/**
		* \~japanese
		* points[]の中で正常に検出された関節点の個数
		*
		* \~english
		* The number of body joint positions properly detected in points[]
		*/
		int num_valid_points;       
		/**
		* \~japanese
		* 時間方向に、このオブジェクトにユニークなIDです。1以上の正の値です。
		*
		* \~english
		* A unique ID for this object in the time direction. An integer value of 1 or more.
		*/
		int id;                     
		/**
		* \~japanese
		* このオブジェクトのオイラー角 yaw, pitch, roll [単位radian]。現在yawのみ対応しています。角度が検出されない場合FLT_MAXが格納されます。
		*
		* \~english
		* Euler angles for this object: yaw, pitch, and roll (in radians). Currently, only yaw is supported. If the angles are not detected, they are set to FLT_MAX.
		*/
		float angle[3];             
	}AILIAPoseEstimatorObjectPose;

	/**
	* \~japanese
	* 構造体フォーマットバージョン
	*
	* \~english
	* Version of the struct format
	*/
	#define AILIA_POSE_ESTIMATOR_OBJECT_FACE_VERSION (1) 
	typedef struct _AILIAPoseEstimatorObjectFace {
		/**
		* \~japanese
		* 検出した顔特徴点。配列インデックスが顔特徴点番号に相当します。
		*
		* \~english
		* Detected human face landmarks. The array index corresponding to a human face landmark number.
		*/
		AILIAPoseEstimatorKeypoint points[AILIA_POSE_ESTIMATOR_FACE_KEYPOINT_CNT]; 
		/**
		* \~japanese
		* このオブジェクトの検出信頼度
		*
		* \~english
		* The confidence of this object
		*/
		float total_score;          
	}AILIAPoseEstimatorObjectFace;

	/**
	* \~japanese
	* 構造体フォーマットバージョン
	*
	* \~english
	* Version of the struct format
	*/
	#define AILIA_POSE_ESTIMATOR_OBJECT_UPPOSE_VERSION (1) 
	typedef struct _AILIAPoseEstimatorObjectUpPose {
		/**
		* \~japanese
		* 検出した関節点。配列インデックスが関節番号に相当します。
		*
		* \~english
		* Detected body joint positions. The array index corresponding to a body joint number.
		*/
		AILIAPoseEstimatorKeypoint points[AILIA_POSE_ESTIMATOR_UPPOSE_KEYPOINT_CNT]; 
		/**
		* \~japanese
		* このオブジェクトの検出信頼度
		*
		* \~english
		* The confidence of this object
		*/
		float total_score;          
		/**
		* \~japanese
		* points[]の中で正常に検出された関節点の個数
		*
		* \~english
		* The number of body joint positions properly detected in points[]
		*/
		int num_valid_points;       
		/**
		* \~japanese
		* 時間方向に、このオブジェクトにユニークなIDです。1以上の正の値です。
		*
		* \~english
		* A unique ID for this object in the time direction. An integer value of 1 or more.
		*/
		int id;                     
		/**
		* \~japanese
		* このオブジェクトのオイラー角 yaw, pitch, roll [単位radian]。現在yawのみ対応しています。角度が検出されない場合FLT_MAXが格納されます。
		*
		* \~english
		* Euler angles for this object: yaw, pitch, and roll (in radians). Currently, only yaw is supported. If the angles are not detected, they are set to FLT_MAX.
		*/
		float angle[3];             
	}AILIAPoseEstimatorObjectUpPose;

	/**
	* \~japanese
	* 構造体フォーマットバージョン
	*
	* \~english
	* Version of the struct format
	*/
	#define AILIA_POSE_ESTIMATOR_OBJECT_HAND_VERSION (1) 
	typedef struct _AILIAPoseEstimatorObjectHand {
		/**
		* \~japanese
		* 検出した関節点。配列インデックスが関節番号に相当します。
		*
		* \~english
		* Detected hand joint positions. The array index corresponding to a hand joint number.
		*/
		AILIAPoseEstimatorKeypoint points[AILIA_POSE_ESTIMATOR_HAND_KEYPOINT_CNT];	
		/**
		* \~japanese
		* このオブジェクトの検出信頼度
		*
		* \~english
		* The confidence of this object
		*/
		float total_score;          
	}AILIAPoseEstimatorObjectHand;

	/****************************************************************
	* 骨格検出・顔特徴点検出API
	**/

	/**
	* \~japanese
	* @brief 検出オブジェクトを作成します。
	* @param pose_estimator 検出オブジェクトポインタ
	* @param net            ネットワークオブジェクトポインタ
	* @param algorithm      検出アルゴリズム (AILIA_POSE_ESTIMATOR_ALGORITHM_*)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   caffemodelとprototxtを読み込んだAILIANetworkから検出オブジェクトを作成します。
	*
	* \~english
	* @brief Creates a estimator instance.
	* @param pose_estimator An estimator instance pointer
	* @param net            The network instance pointer
	* @param algorithm      Estimation algorithm(AILIA_POSE_ESTIMATOR_ALGORITHM_*)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	* @details
	*   This function creates an estimator instance from AILIANetwork when reading caffemodel and prototxt.
	*/
	int AILIA_API ailiaCreatePoseEstimator(struct AILIAPoseEstimator **pose_estimator, struct AILIANetwork *net, unsigned int algorithm);

	/**
	* \~japanese
	* @brief 検出オブジェクトを破棄します。
	* @param pose_estimator 検出オブジェクトポインタ
	*
	* \~english
	* @brief Destroys the estimator instance.
	* @param pose_estimator An estimator instance pointer
	*/
	void AILIA_API ailiaDestroyPoseEstimator(struct AILIAPoseEstimator *pose_estimator);

	/**
	* \~japanese
	* @brief 検出閾値を設定します。
	* @param pose_estimator              検出オブジェクトポインタ
	* @param threshold                   検出閾値 0.0以上1.0以下の値で、値が小さいほど検出しやすくなります。
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Set the detection threshold.
	* @param pose_estimator              An estimator instance pointer
	* @param threshold                   The detection threshold (for example, 0.1f) (The smaller it is, the easier the detection will be and the more detected objects found.)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaPoseEstimatorSetThreshold(struct AILIAPoseEstimator *pose_estimator, float threshold);

	/**
	* \~japanese
	* @brief 骨格検出・顔特徴点検出を行います。
	* @param pose_estimator              検出オブジェクトポインタ
	* @param src                         画像データ(32bpp)
	* @param src_stride                  1ラインのバイト数
	* @param src_width                   画像幅
	* @param src_height                  画像高さ
	* @param src_format                  画像形式 (AILIA_IMAGE_FORMAT_*)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Performs human pose estimation and human face landmarks extraction.
	* @param pose_estimator              An estimator instance pointer
	* @param src                         Image data (32 bpp)
	* @param src_stride                  The number of bytes in 1 line
	* @param src_width                   Image width
	* @param src_height                  Image height
	* @param src_format                  Image format (AILIA_IMAGE_FORMAT_*)
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaPoseEstimatorCompute(struct AILIAPoseEstimator *pose_estimator, const void *src, unsigned int src_stride, unsigned int src_width, unsigned int src_height, unsigned int src_format);

	/**
	* \~japanese
	* @brief 検出結果の数を取得します。
	* @param pose_estimator  検出オブジェクトポインタ
	* @param obj_count       オブジェクト数  顔特徴点の場合は1または0となります。
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the number of detection results.
	* @param pose_estimator  An estimator instance pointer
	* @param obj_count       The number of objects. Set to 1 or 0 for human face landmarks.
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaPoseEstimatorGetObjectCount(struct AILIAPoseEstimator *pose_estimator, unsigned int *obj_count);

	/**
	* \~japanese
	* @brief 骨格検出結果を取得します。
	* @param pose_estimator  検出オブジェクトポインタ
	* @param obj             オブジェクト情報
	* @param obj_idx         オブジェクトインデックス
	* @param version         AILIA_POSE_ESTIMATOR_OBJECT_POSE_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the results of the human pose estimation.
	* @param pose_estimator  An estimator instance pointer
	* @param obj             Object information
	* @param obj_idx         Object index
	* @param version         AILIA_POSE_ESTIMATOR_OBJECT_POSE_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaPoseEstimatorGetObjectPose(struct AILIAPoseEstimator *pose_estimator, AILIAPoseEstimatorObjectPose* obj, unsigned int obj_idx, unsigned int version);

	/**
	* \~japanese
	* @brief 顔特徴点検出結果を取得します。
	* @param pose_estimator  検出オブジェクトポインタ
	* @param obj             オブジェクト情報
	* @param obj_idx         オブジェクトインデックス 必ず 0 を指定してください。
	* @param version         AILIA_POSE_ESTIMATOR_OBJECT_FACE_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the results of the human face landmarks extraction.
	* @param pose_estimator  An estimator instance pointer
	* @param obj             Object information
	* @param obj_idx         Object index. Ensure that 0 is specified.
	* @param version         AILIA_POSE_ESTIMATOR_OBJECT_FACE_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaPoseEstimatorGetObjectFace(struct AILIAPoseEstimator *pose_estimator, AILIAPoseEstimatorObjectFace* obj, unsigned int obj_idx, unsigned int version);

	/**
	* \~japanese
	* @brief UpPose 認識結果を取得します。
	* @param pose_estimator  検出オブジェクトポインタ
	* @param obj             オブジェクト情報
	* @param obj_idx         オブジェクトインデックス
	* @param version         AILIA_POSE_ESTIMATOR_OBJECT_POSE_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the results of the human up pose estimation.
	* @param pose_estimator  An estimator instance pointer
	* @param obj             Object information
	* @param obj_idx         Object index
	* @param version         AILIA_POSE_ESTIMATOR_OBJECT_UPPOSE_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaPoseEstimatorGetObjectUpPose(struct AILIAPoseEstimator *pose_estimator, AILIAPoseEstimatorObjectUpPose* obj, unsigned int obj_idx, unsigned int version);

	/**
	* \~japanese
	* @brief Hand 認識結果を取得します。
	* @param pose_estimator  検出オブジェクトポインタ
	* @param obj             オブジェクト情報
	* @param obj_idx         オブジェクトインデックス 必ず 0 を指定してください。
	* @param version         AILIA_POSE_ESTIMATOR_OBJECT_HAND_VERSION
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Gets the results of the human hand estimation.
	* @param pose_estimator  An estimator instance pointer
	* @param obj             Object information
	* @param obj_idx         Object index. Ensure that 0 is specified.
	* @param version         AILIA_POSE_ESTIMATOR_OBJECT_HAND_VERSION
	* @return
	*   If this function is successful, it returns  \ref AILIA_STATUS_SUCCESS , or an error code otherwise.
	*/
	int AILIA_API ailiaPoseEstimatorGetObjectHand(struct AILIAPoseEstimator *pose_estimator, AILIAPoseEstimatorObjectHand* obj, unsigned int obj_idx, unsigned int version);

#ifdef __cplusplus
}
#endif
#endif /* !defined(INCLUDED_AILIA_POSE_ESTIMATOR) */
