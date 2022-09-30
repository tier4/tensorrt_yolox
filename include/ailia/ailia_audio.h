/**
 * \~japanese
 * @file ailia_audio.h
 * @brief 音響信号処理用ライブラリ
 * @copyright AXELL CORPORATION, ax Inc.
 * @date 2021/07/28
 *
 * \~english
 * @file ailia_audio.h
 * @brief audio processing library
 * @copyright AXELL CORPORATION, ax Inc.
 * @date 2021/07/28
 */

#if       !defined(INCLUDED_AILIA_AUDIO)
#define            INCLUDED_AILIA_AUDIO

#if !defined(AILIA_API)
	#if defined(_MSC_VER) && !defined(_WIN64)
		#define AILIA_API __stdcall
	#else
		#define AILIA_API
	#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

	/**
	* \~japanese
	* @def AILIA_AUDIO_WIN_TYPE_HANN
	* @brief 窓関数に hann 窓を使う 
	*
	* \~english
	* @def AILIA_AUDIO_WIN_TYPE_HANN
	* @brief use a Hann window function 
	*/
	#define AILIA_AUDIO_WIN_TYPE_HANN                   (1) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_WIN_TYPE_HAMMING
	* @brief 窓関数に hamming 窓を使う 
	*
	* \~english
	* @def AILIA_AUDIO_WIN_TYPE_HAMMING
	* @brief use a Hamming window function 
	*/
	#define AILIA_AUDIO_WIN_TYPE_HAMMING                (2) 

	/**
	* \~japanese
	* @def AILIA_AUDIO_STFT_CENTER_NONE
	* @brief STFT の際、前後に padding を入れない 
	*
	* \~english
	* @def AILIA_AUDIO_STFT_CENTER_NONE
	* @brief for the STFT, do not insert padding before and after 
	*/
	#define AILIA_AUDIO_STFT_CENTER_NONE                (0) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_STFT_CENTER_ENABLE
	* @brief STFT の際、sample_n の前後に fft_n/2 の padding(reflect) を入れる 
	*
	* \~english
	* @def AILIA_AUDIO_STFT_CENTER_ENABLE
	* @brief for the STFT, insert a padding (reflect) of fft_n/2 before and after the sample_n samples 
	*/
	#define AILIA_AUDIO_STFT_CENTER_ENABLE              (1) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_STFT_CENTER_SCIPY_DEFAULT
	* @brief STFT の際、sample_n の前後に fft_n/2 の padding(zero) を入れ、さらにhop_n処理単位になるように後方padding(zero)で合わせる 
	*
	* \~english
	* @def AILIA_AUDIO_STFT_CENTER_SCIPY_DEFAULT
	* @brief for the STFT, insert a padding (zeros) of fft_n/2 before and after the sample_n samples, and also pad at the end with zeros to process in units of hop_n 
	*/
	#define AILIA_AUDIO_STFT_CENTER_SCIPY_DEFAULT       (2) 

	/**
	* \~japanese
	* @def AILIA_AUDIO_FFT_NORMALIZE_NONE
	* @brief FFT の出力を正規化しない 
	*
	* \~english
	* @def AILIA_AUDIO_FFT_NORMALIZE_NONE
	* @brief Do not normalize the FFT output 
	*/
	#define AILIA_AUDIO_FFT_NORMALIZE_NONE              (0) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_FFT_NORMALIZE_LIBROSA_COMPAT
	* @brief FFT の出力を librosa 互換で正規化する 
	*
	* \~english
	* @def AILIA_AUDIO_FFT_NORMALIZE_LIBROSA_COMPAT
	* @brief Normalize the FFT output in a way compatible with librosa 
	*/
	#define AILIA_AUDIO_FFT_NORMALIZE_LIBROSA_COMPAT    (1) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_FFT_NORMALIZE_PYTORCH_COMPAT
	* @brief FFT の出力を PyTorch 互換で正規化する 
	*
	* \~english
	* @def AILIA_AUDIO_FFT_NORMALIZE_PYTORCH_COMPAT
	* @brief Normalize the FFT output in a way compatible with PyTorch 
	*/
	#define AILIA_AUDIO_FFT_NORMALIZE_PYTORCH_COMPAT    (1) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_FFT_NORMALIZE_SCIPY_COMPAT
	* @brief FFT の出力を SciPy 互換で正規化する 
	*
	* \~english
	* @def AILIA_AUDIO_FFT_NORMALIZE_SCIPY_COMPAT
	* @brief Normalize the FFT output in a way compatible with SciPy 
	*/
	#define AILIA_AUDIO_FFT_NORMALIZE_SCIPY_COMPAT      (2) 

	/**
	* \~japanese
	* @def AILIA_AUDIO_MEL_NORMALIZE_NONE
	* @brief MEL スペクトログラムの出力を正規化しない 
	*
	* \~english
	* @def AILIA_AUDIO_MEL_NORMALIZE_NONE
	* @brief Do not normalize the output of the mel spectrogram 
	*/
	#define AILIA_AUDIO_MEL_NORMALIZE_NONE              (0) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_MEL_NORMALIZE_ENABLE
	* @brief MEL スペクトログラムの出力を正規化する 
	*
	* \~english
	* @def AILIA_AUDIO_MEL_NORMALIZE_ENABLE
	* @brief Normalize the output of the mel spectrogram 
	*/
	#define AILIA_AUDIO_MEL_NORMALIZE_ENABLE            (1) 

	/**
	* \~japanese
	* @def AILIA_AUDIO_MEL_SCALE_FORMULA_HTK
	* @brief MEL 尺度を HTK formula で求める (PyTorch 互換) 
	*
	* \~english
	* @def AILIA_AUDIO_MEL_SCALE_FORMULA_HTK
	* @brief Get the mel scale from the HTK formula (PyTorch compatible) 
	*/
	#define AILIA_AUDIO_MEL_SCALE_FORMULA_HTK           (1) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_MEL_SCALE_FORMULA_SLANYE
	* @brief MEL 尺度を Slanye's formula で求める (librosa デフォルト互換) 
	*
	* \~english
	* @def AILIA_AUDIO_MEL_SCALE_FORMULA_SLANYE
	* @brief Get the mel scale from the Slanye's formula (compatible with the default of librosa) 
	*/
	#define AILIA_AUDIO_MEL_SCALE_FORMULA_SLANYE        (0) 

	/**
	* \~japanese
	* @def AILIA_AUDIO_PHASE_FORM_COMPLEX
	* @brief 位相を複素数形式で出力する (librosa デフォルト互換) 
	*
	* \~english
	* @def AILIA_AUDIO_PHASE_FORM_COMPLEX
	* @brief Output the phase in complex format (compatible with the default of librosa) 
	*/
	#define AILIA_AUDIO_PHASE_FORM_COMPLEX              (1) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_PHASE_FORM_REAL
	* @brief 位相を実数形式で出力する (PyTorch デフォルト互換) 
	*
	* \~english
	* @def AILIA_AUDIO_PHASE_FORM_REAL
	* @brief Output the phase in complex format (compatible with the default of PyTorch) 
	*/
	#define AILIA_AUDIO_PHASE_FORM_REAL                 (0) 

	/**
	* \~japanese
	* @def AILIA_AUDIO_FILTFILT_PAD_NONE
	* @brief ゼロ位相フィルタ処理の際、padding をしない 
	*
	* \~english
	* @def AILIA_AUDIO_FILTFILT_PAD_NONE
	* @brief During zero-phase filtering, do not pad 
	*/
	#define AILIA_AUDIO_FILTFILT_PAD_NONE               (0) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_FILTFILT_PAD_ODD
	* @brief ゼロ位相フィルタ処理の際、padding はodd 
	*
	* \~english
	* @def AILIA_AUDIO_FILTFILT_PAD_ODD
	* @brief During zero-phase filtering, pad with an odd reflection (substract the reflected values from two times the edge value) 
	*/
	#define AILIA_AUDIO_FILTFILT_PAD_ODD                (1) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_FILTFILT_PAD_EVEN
	* @brief ゼロ位相フィルタ処理の際、padding はeven(reflect) 
	*
	* \~english
	* @def AILIA_AUDIO_FILTFILT_PAD_EVEN
	* @brief During zero-phase filtering, pad with an even reflection (normal reflection) 
	*/
	#define AILIA_AUDIO_FILTFILT_PAD_EVEN               (2) 
	/**
	* \~japanese
	* @def AILIA_AUDIO_FILTFILT_PAD_CONSTANT
	* @brief ゼロ位相フィルタ処理の際、padding は端値 
	*
	* \~english
	* @def AILIA_AUDIO_FILTFILT_PAD_CONSTANT
	* @brief During zero-phase filtering, pad using the edge value 
	*/
	#define AILIA_AUDIO_FILTFILT_PAD_CONSTANT           (3) 

	/**
	* \~japanese
	* @brief 入力値を対数スケールに変換します。
	* @param dst 出力データポインタ、float 型、長さ src_n
	* @param src 入力データポインタ、float 型、長さ src_n
	* @param src_n 計算対象の要素数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   dst = log_e(1.0 + src) を計算します。
	*
	* \~english
	* @brief Convert the input values to a logarithmic scale.
	* @param dst pointer to the output data, of float format, and of length src_n
	* @param src pointer to the input data, of float format, and of length src_n
	* @param src_n number of elements to be calculated
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   dst = log_e(1.0 + src)
	*/
	int AILIA_API ailiaAudioLog1p(void* dst, const void* src, int src_n);


	/**
	* \~japanese
	* @brief 非負の入力値をデシベルスケールに変換します。
	* @param dst 出力データポインタ、float 型、長さ src_n
	* @param src 入力データポインタ、float 型、要素数 src_n
	* @param src_n 計算対象の要素数
	* @param top_db 出力の最大値から出力下限の閾値までを定める値 (>= 0.0)、負数の場合は処理は閾値を設定しない
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   librosa.power_to_dbと互換性があります。
	*
	* \~english
	* @brief Convert non-negative input values to decibel scale.
	* @param dst pointer to the output data, of float format, and of length src_n
	* @param src pointer to the input data, of float format, and of length src_n
	* @param src_n number of elements to be calculated
	* @param top_db float >= 0.0
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   Output compatible with librosa.power_to_db.
	*   dst = trimlow( 10 * log10(src / ref) )
	*   where ref is the max of 1e-10 and of positive values of src,
	*   and trimlow(), if top_db > 0, trims all values inferior to (- top_db) and replaces them by (- top_db)),
	*   else, trimlow() does nothing.
	*/
	int AILIA_API ailiaAudioConvertPowerToDB(void* dst, const void* src, int src_n, float top_db);


	/**
	* \~japanese
	* @brief STFTで生成される時間フレーム長を取得します。
	* @param frame_n フレーム長出力先ポインタ
	* @param sample_n STFTを適用するサンプル数
	* @param fft_n FFT点数
	* @param hop_n 窓のシフト数
	* @param center AILIA_AUDIO_STFT_CENTER_* のいずれか
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   STFT実行前のバッファサイズの決定に使用します。
	*    \ref AILIA_AUDIO_STFT_CENTER_NONE  の場合 hop_n ずつ区切り、sample_n の前後に padding を行いません。
	*    \ref AILIA_AUDIO_STFT_CENTER_ENABLE  の場合 sample_n の前後に fft_n/2 の padding(reflect) を行います。
	*    \ref AILIA_AUDIO_STFT_CENTER_SCIPY_DEFAULT の場合、sample_n の前後に fft_n/2 の padding(zero) を行い、hop_nの倍数になるようにpadding(zero)を行います
	*
	* \~english
	* @brief Get the number of frames generated by the STFT.
	* @param frame_n pointer to the destination where to write the output (the number of frames)
	* @param sample_n count of samples on which the STFT is performed
	* @param fft_n size of the FFT at each frame (i.e. number of frequency bins at each frame)
	* @param hop_n stride of each window shift (in number of samples). This is the quantum of time for the time axis of the STFT output.
	* @param center any of the AILIA_AUDIO_STFT_CENTER_* constants
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   Before executing the STFT, use this function to determine the space required for the output buffer.
	*   If  \ref AILIA_AUDIO_STFT_CENTER_NONE  is used, the sample_n samples are cut in packets of size hop_n, and no padding occurs before the first sample nor after the last sample.
	*   If  \ref AILIA_AUDIO_STFT_CENTER_ENABLE  is used, a reflection padding of length fft_n/n is performed before the first sample and after the last sample.
	*   If  \ref AILIA_AUDIO_STFT_CENTER_ENABLE  is used, a zero padding of length fft_n/n is performed before the first sample and after the last sample, and moreover an additional zero padding is performed to ensure that the total length is a multiple of hop_n.
	*/
	int AILIA_API ailiaAudioGetFrameLen(int* frame_n, int sample_n, int fft_n, int hop_n, int center);


	/**
	* \~japanese
	* @brief ISTFTで生成されるサンプル数を取得します。
	* @param sample_n サンプル数出力先ポインタ
	* @param frame_n STFTデータの時間フレーム長
	* @param fft_n FFT点数
	* @param hop_n 窓のシフト数
	* @param center AILIA_AUDIO_STFT_CENTER_* のいずれか
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   ISTFT実行前のバッファサイズの決定に使用します。
	*    \ref AILIA_AUDIO_STFT_CENTER_NONE  の場合 前後の切り捨てを行いません。
	*    \ref AILIA_AUDIO_STFT_CENTER_NONE  以外の場合 前後の切り捨てを行います。
	*
	* \~english
	* @brief Get the number of samples generated by the ISTFT.
	* @param sample_n pointer to the destination where to write the output (the number of samples)
	* @param frame_n length of the STFT data, expressed in number of frames
	* @param fft_n size of the FFT at each frame (i.e. number of frequency bins at each frame)
	* @param hop_n stride of each window shift (in number of samples). This is the quantum of time for the time axis of the STFT output.
	* @param center any of the AILIA_AUDIO_STFT_CENTER_* constants
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   Before executing the ISTFT, use this function to determine the space required for the output buffer.
	*   If  \ref AILIA_AUDIO_STFT_CENTER_NONE  is used, no truncation is performed at the beginning nor at the end.
	*   If  \ref AILIA_AUDIO_STFT_CENTER_NONE  is not used, a truncation is performed at the beginning and at the end.
	*/
	int AILIA_API ailiaAudioGetSampleLen(int* sample_n, int frame_n, int freq_n, int hop_n, int center);


	/**
	* \~japanese
	* @brief 窓関数の係数を取得します。
	* @param dst 出力データのポインタ、float 型、要素数 window_n
	* @param window_n 窓の長さ（サンプル数）
	* @param win_type 窓関数の種類、AILIA_AUDIO_WIN_TYPE_* のいずれか
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   窓関数はhann窓とhamming窓のみ対応しています。
	*
	* \~english
	* @brief Get the window function.
	* @param dst pointer to the output data, of float format, and of length window_n
	* @param window_n length of the window (in number of samples)
	* @param win_type type of the window function: any of the AILIA_AUDIO_WIN_TYPE_* constants
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   Only the Hann and the Hamming window functions are supported.
	*/
	int AILIA_API ailiaAudioGetWindow(void* dst, int window_n, int win_type);


	/**
	* \~japanese
	* @brief FFTを実行します。
	* @param dst 出力データのポインタ、float 型、外側から fft_n, 2(実部、虚部) 順のメモリレイアウト
	* @param src 入力データのポインタ、float 型、要素数 fft_n
	* @param fft_n FFT点数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   FFT点数が2の累乗の場合、高速アルゴリズムで動作します。
	*   出力データは実部と虚部の交互信号であり、長さは fft_n*2 です。
	*
	* \~english
	* @brief Execute the FFT.
	* @param dst pointer to the output data, of float format, of length 2*fft_n, and which memory layout is a sequence of fft_n pairs [real part, imaginary part]. Memory layout, using the row-major convention: (fft_n, 2).
	* @param src pointer to the input data, of float format, and of length fft_n
	* @param fft_n count of FFT values (i.e. of frequency bins)
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   If fft_n is a power of 2, this function uses a faster algorithm.
	*   As the output data alternates real and imaginary parts, its length is 2*fft_n.
	*/
	int AILIA_API ailiaAudioFFT(void* dst, const void* src, int fft_n);


	/**
	* \~japanese
	* @brief IFFTを実行します。
	* @param dst 出力データのポインタ、float 型、外側から fft_n, 2(実部、虚部) 順のメモリレイアウト
	* @param src 入力データのポインタ、float 型、外側から fft_n, 2(実部、虚部) 順のメモリレイアウト
	* @param fft_n FFT点数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   FFT点数が2の累乗の場合、高速アルゴリズムで動作します。
	*   出力データは実部と虚部の交互信号であり、長さは fft_n*2 です。
	*
	* \~english
	* @brief Execute the IFFT.
	* @param dst pointer to the output data, of float format, of length 2*fft_n, and which memory layout is a sequence of fft_n pairs [real part, imaginary part]. Memory layout, using the row-major convention: (fft_n, 2).
	* @param src pointer to the input data, of float format, of length 2*fft_n, and which memory layout is a sequence of fft_n pairs [real part, imaginary part]. Memory layout, using the row-major convention: (fft_n, 2).
	* @param fft_n count of FFT values (i.e. of frequency bins)
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   If fft_n is a power of 2, this function uses a faster algorithm.
	*   As the output data alternates real and imaginary parts, its length is 2*fft_n.
	*/
	int AILIA_API ailiaAudioIFFT(void* dst, const void* src, int fft_n);


	/**
	* \~japanese
	* @brief 音響信号からスペクトログラムを生成します。
	* @param dst 出力データのポインタ、float 型、外側から freq_n, frame_n, 2(複素数: 実部, 虚部) 順のメモリレイアウト
	* @param src 入力データのポインタ、float 型、要素数 sample_n
	* @param sample_n 入力データのサンプル数
	* @param fft_n FFT点数
	* @param hop_n フレームのシフト数
	* @param win_n 窓関数の長さ
	* @param win_type 窓関数の種類、AILIA_AUDIO_WIN_TYPE_* のいずれか
	* @param max_frame_n 出力データの時間フレーム数の最大値
	* @param center 入力データの前後へのパディングの有無、AILIA_AUDIO_STFT_CENTER_* のいずれか
	* @param power スペクトログラムの乗数（>= 0.0） 0.0: 複素スペクトログラム、1.0: 振幅スペクトログラム、2.0: パワースペクトログラム、その他: 任意の累乗値の出力に相当
	* @param norm_type FFT後の正規化処理、AILIA_AUDIO_FFT_NORMALIZE_* のいずれか
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   時間フレームごとにFFT→正規化処理→累乗（振幅・パワーに変換）の順で処理を実行します。
	*   出力データは実部と虚部の交互信号であり、長さは(fft_n/2+1)*時間フレーム長*2です。
	*   powerが0.0以外の場合は虚部の値を全て0.0として出力します。
	*
	* \~english
	* @brief Generate the spectrogram from the audio signal.
	* @param dst pointer to the output data, of float format, of length (2 * freq_n * frame_n), and which memory layout is a sequence of pairs [real part, imaginary part]. (where freq_n = fft_n/2+1). Memory layout, using the row-major convention: (freq_n, frame_n, 2).
	* @param src pointer to the input data, of float format, and of length sample_n
	* @param sample_n count of samples in the input data
	* @param fft_n size of the FFT at each frame (i.e. number of frequency bins at each frame)
	* @param hop_n stride of each window shift (in number of samples). This is the size of the time increment for the spectrogram.
	* @param win_n size of the window function
	* @param win_type type of the window function: any of the AILIA_AUDIO_WIN_TYPE_* constants
	* @param max_frame_n maximum value of the time frame index in the outputted data
	* @param center whether to pad or not (and the type of padding) before and after the input data: any of the AILIA_AUDIO_STFT_CENTER_* constants
	* @param power exponent to apply to the spectrogram (> = 0.0). A special case is for 0.0: complex spectrogram. For other cases the amplitude is just exponentiated accordingly: 1.0: amplitude spectrogram, 2.0: power spectrogram, etc, any other positive exponent value is allowed.
	* @param norm_type normalization after the FFT: any of the AILIA_AUDIO_FFT_NORMALIZE_* constants
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   For each time frame, the operations are processed in this order: FFT -> normalization -> power exponentiation.
	*   As the output data alternates real and imaginary parts, its length is 2*(fft_n/2+1)*frame_n. (where frame_n is the number of time frames outputted)
	*   When the power argument is a non-zero value, all the complex parts are set to 0 in the output.
	*/
	int AILIA_API ailiaAudioGetSpectrogram(void* dst, const void* src, int sample_n, int fft_n, int hop_n, int win_n, int win_type, int max_frame_n, int center, float power, int norm_type);


	/**
	* \~japanese
	* @brief 複素スペクトログラムから音響信号を生成します。
	* @param dst 出力データのポインタ、float 型、要素数 sample_n
	* @param src 入力データのポインタ、float 型、外側から freq_n, frame_n, 2(複素数: 実部, 虚部) 順のメモリレイアウト
	* @param frame_n 入力データの時間フレーム数
	* @param freq_n 周波数（fft_n/2+1）
	* @param hop_n フレームのシフト数
	* @param win_n 窓関数の長さ
	* @param win_type 窓関数の種類、AILIA_AUDIO_WIN_TYPE_* のいずれか
	* @param max_sample_n 出力データのサンプル数の最大値
	* @param center 入力データ生成時の前後へのパディングの有無、AILIA_AUDIO_STFT_CENTER_* のいずれか
	* @param norm_type 入力データ生成時の正規化処理、AILIA_AUDIO_FFT_NORMALIZE_* のいずれか
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   時間フレームごとにIFFTを行い、最後に正規化処理を実行します。
	*   複素スペクトログラムのみに対応しています。
	*
	* \~english
	* @brief Generate an audio signal from a complex spectrogram.
	* @param dst pointer to the output data, of float format, and of length sample_n
	* @param src pointer to the input data, of float format, of length (2 * freq_n * frame_n), and which memory layout is a sequence of pairs [real part, imaginary part]. Memory layout, using the row-major convention: (freq_n, frame_n, 2).
	* @param frame_n number of time frames in the input data
	* @param freq_n number of frequencies bins for each time frame (freq_n = fft_n/2+1)
	* @param hop_n step size of the time frame increment (expressed in number of samples) for the inputted spectrogram.
	* @param win_n size of the window function
	* @param win_type type of the window function: any of the AILIA_AUDIO_WIN_TYPE_* constants
	* @param max_sample_n maximum value of the sample index in the outputted data
	* @param center whether padding (before and after) was used or not (and its type) during the generation of the input data: any of the AILIA_AUDIO_STFT_CENTER_* constants
	* @param norm_type normalization type that was used during the generation of the input data: any of the AILIA_AUDIO_FFT_NORMALIZE_* constants
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   For each time frame the normalization is executed at the end of the IFFT.
	*   Only accepts a complex spectrogram in input.
	*/
	int AILIA_API ailiaAudioGetInverseSpectrogram(void* dst, const void* src, int frame_n, int freq_n, int hop_n, int win_n, int win_type, int max_sample_n, int center, int norm_type);


	/**
	* \~japanese
	* @brief メルフィルタバンクの係数を計算します。
	* @param dst 出力データのポインタ、float 型、外側から mel_n, freq_n  順のメモリレイアウト
	* @param freq_n 周波数のインデックス数
	* @param f_min 周波数の最小値
	* @param f_max 周波数の最大値
	* @param mel_n メル周波数のインデックス数（ < freq_n）
	* @param sample_rate サンプリング周波数
	* @param mel_norm 出力される係数の正規化の有無、AILIA_AUDIO_MEL_NORMALIZE_* のいずれか
	* @param mel_formula MEL尺度の形式、AILIA_AUDIO_MEL_SCALE_FORMULA_* のいずれか
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Create a mel filter-bank.
	* @param dst pointer to the output data, of float format, and of length (mel_n * freq_n). (memory layout, using the row-major convention: (mel_n, freq_n))
	* @param freq_n number of frequency indices for the FFT (1+fft_n/2)
	* @param f_min lowest frequency
	* @param f_max highest frequency
	* @param mel_n number of mel frequency bins in the output (< freq_n)
	* @param sample_rate sampling rate for the signal that will be inputted to this filter
	* @param mel_norm whether to normalize the output (and the type of the normalization): any of the AILIA_AUDIO_MEL_NORMALIZE_* constants
	* @param mel_formula mel scale format: any of the AILIA_AUDIO_MEL_SCALE_FORMULA_* constants
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	*/
	int AILIA_API ailiaAudioGetFBMatrix(void* dst, const int freq_n,  float f_min, float f_max, int mel_n, int sample_rate, int mel_norm, int mel_formula);


	/**
	* \~japanese
	* @brief 音響信号からメルスペクトログラムを生成します。
	* @param dst 出力データのポインタ、float 型、外側から mel_n, frame_n 順のメモリレイアウト
	* @param src 入力データのポインタ、float 型、モノラル PCM データ
	* @param sample_n 入力データのサンプル数
	* @param sample_rate サンプリング周波数
	* @param fft_n FFT点数
	* @param hop_n フレームのシフト数
	* @param win_n 1フレームに含むサンプル数
	* @param win_type 窓関数の種類、AILIA_AUDIO_WIN_TYPE_* のいずれか
	* @param max_frame_n 出力データの時間フレーム数の最大値
	* @param center 入力データの前後へのパディングの有無、AILIA_AUDIO_STFT_CENTER_* のいずれか
	* @param power スペクトログラムの乗数（ > 0.0）1.0: 振幅スペクトログラム、2.0: パワースペクトログラム、その他: 任意の累乗値の出力に相当
	* @param fft_norm_type FFT後の正規化処理、AILIA_AUDIO_FFT_NORMALIZE_* のいずれか
	* @param f_min 周波数の最小値
	* @param f_max 周波数の最大値
	* @param mel_n メル周波数のインデックス数（ < freq_n）
	* @param mel_norm_type MELスペクトログラムの正規化の有無、AILIA_AUDIO_MEL_NORMALIZE_* のいずれか
	* @param mel_formula MEL尺度の形式、AILIA_AUDIO_MEL_SCALE_FORMULA_* のいずれか
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   時間フレームごとにFFT(STFT)→正規化処理→累乗（振幅・パワーに変換→メルフィルタバンクの係数取得→メル尺度への変換 の順で処理を実行します。
	*   出力データは実数の信号であり、長さはmel_n*時間フレーム長です。
	*
	* \~english
	* @brief Generate the mel spectrogram from the audio signal.
	* @param dst pointer to the output data, of float format, and of length (mel_n * frame_n) (with frame_n the number of time frames outputted). (memory layout, using the row-major convention: (mel_n, frame_n))
	* @param src pointer to the input data, of float format, monoral PCM audio data.
	* @param sample_n count of samples in the input data
	* @param sample_rate sampling rate of the input signal
	* @param fft_n number of FFT components
	* @param hop_n stride of each window shift (in number of samples). This is the size of the time increment for the spectrogram.
	* @param win_n size of the window function (in number of samples)
	* @param win_type type of the window function: any of the AILIA_AUDIO_WIN_TYPE_* constants
	* @param max_frame_n maximum value of the time frame index in the outputted data
	* @param center whether to pad or not (and the type of padding) before and after the input data: any of the AILIA_AUDIO_STFT_CENTER_* constants
	* @param power exponent to apply to the spectrogram (> 0.0). 1.0: amplitude spectrogram, 2.0: power spectrogram, etc, any other positive exponent value is allowed.
	* @param fft_norm_type normalization after the FFT: any of the AILIA_AUDIO_FFT_NORMALIZE_* constants
	* @param f_min lowest frequency
	* @param f_max highest frequency
	* @param mel_n number of mel frequency bins in the output (< freq_n)
	* @param mel_norm whether to normalize the mel spectrogram (and the type of the normalization): any of the AILIA_AUDIO_MEL_NORMALIZE_* constants
	* @param mel_formula mel scale format: any of the AILIA_AUDIO_MEL_SCALE_FORMULA_* constants
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   For each time frame, the operations are processed in this order:
	*   FFT(STFT) -> normalization -> power exponentiation -> get the mel filter-bank coefficients -> convert to the mel scale.
	*   The output is real values, and its length is mel_n*frame_n (with frame_n the number of time frames outputted).
	*/
	int AILIA_API ailiaAudioGetMelSpectrogram(void* dst, const void* src, int sample_n, int sample_rate, int fft_n, int hop_n, int win_n, int win_type, int max_frame_n, int center, float power, int fft_norm_type, float f_min, float f_max, int mel_n, int mel_norm_type, int mel_formula);


	/**
	* \~japanese
	* @brief スペクトログラムから振幅と位相を計算します。
	* @param dst_mag 振幅の出力先ポインタ、外側から freq_n, frame_n 順のメモリレイアウト
	* @param dst_phase 位相の出力先ポインタ、外側から freq_n, frame_n, 2(実部、虚部) 順のメモリレイアウト
	* @param src 入力データのポインタ、frame_n, freq_n, 2(実部、虚部) 順のメモリレイアウト
	* @param freq_n 周波数のインデックス数
	* @param frame_n 時間フレームの数
	* @param power 振幅スペクトルの乗数 ( > 0.0)、1.0: 振幅スペクトログラム、2.0: パワースペクトログラムに相当
	* @param phase_form 位相の出力形式、AILIA_AUDIO_PHASE_FORM_* のいずれか
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   librosaのデフォルト値と互換の条件: phase_form =  \ref AILIA_AUDIO_PHASE_FORM_COMPLEX , power = 1.0
	*   PyTorchのデフォルト値と互換の条件: phase_form =  \ref AILIA_AUDIO_PHASE_FORM_REAL , power = 1.0
	*   phase_formによってdst_phaseの出力が変わります。
	*   -  \ref AILIA_AUDIO_PHASE_FORM_COMPLEX  : 実部と虚部の交互信号、サイズは freq_n * frame_n * 2
	*   -  \ref AILIA_AUDIO_PHASE_FORM_REAL  : 実部のみの信号、サイズは freq_n * frame_n
	*
	* \~english
	* @brief Get the amplitude and the phase from the spectrogram.
	* @param dst_mag pointer to the outputted amplitudes, an array of length (freq_n * frame_n). (memory layout, using the row-major convention: (freq_n, frame_n))
	* @param dst_phase pointer to the outputted phases, an array of length (2 * freq_n * frame_n) (sequence of complex pairs [real part, imaginary part]). (memory layout, using the row-major convention: (freq_n, frame_n, 2))
	* @param src pointer to the input data, of length (2 * frame_n * freq_n) (a sequence of complex pairs [real, imaginary]). (memory layout, using the row-major convention: (frame_n, freq_n, 2))
	* @param freq_n number of frequency indices
	* @param frame_n number of time frames
	* @param power exponent to apply to the spectrogram (> 0.0). 1.0: amplitude spectrogram, 2.0: power spectrogram, etc, any other positive exponent value is allowed.
	* @param phase_form format of the outputted phase: any of the AILIA_AUDIO_PHASE_FORM_* constants
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   To be compatible with librosa, use: phase_form =  \ref AILIA_AUDIO_PHASE_FORM_COMPLEX , power = 1.0
	*   To be compatible with PyTorch, use: phase_form =  \ref AILIA_AUDIO_PHASE_FORM_REAL , power = 1.0
	*   The dst_phase output depends on phase_form:
	*   -  \ref AILIA_AUDIO_PHASE_FORM_COMPLEX  : signal with real and imaginary parts, of size (freq_n * frame_n * 2)
	*   -  \ref AILIA_AUDIO_PHASE_FORM_REAL  : real signal, of size (freq_n * frame_n)
	*/
	int AILIA_API ailiaAudioMagPhase(void* dst_mag, void* dst_phase, const void* src, int freq_n, int frame_n, float power, int phase_form);


	/**
	* \~japanese
	* @brief 実数の信号に対して標準化を実行します。
	* @param dst 出力データのポインタ、float 型、要素数 src_n
	* @param src 入力データのポインタ、float 型、要素数 src_n
	* @param src_n 入力データの要素数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   入力データの平均0、分散1になるよう標準化を行う。
	*   dst = (src - mean(src)) / std(src)
	*
	* \~english
	* @brief Standardize a real signal.
	* @param dst pointer to the output data, of float format, and of length src_n
	* @param src pointer to the input data, of float format, and of length src_n
	* @param src_n length of the input data
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   Standardize the input data so that its average value becomes 0 and its variance 1.
	*   dst = (src - mean(src)) / std(src)
	*/
	int AILIA_API ailiaAudioStandardize(void* dst, const void* src, const int src_n);


	/**
	* \~japanese
	* @brief 複素数のノルムを算出します
	* @param dst 出力データのポインタ、float 型、要素数 src_n
	* @param src 入力データのポインタ、float 型、外側から src_n, 2(実部、虚部) 順のメモリレイアウト
	* @param src_n 入力データの要素数
	* @param power 累乗値( > 0.0 )、1.0で複素数絶対値に相当
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   入力データのノルムを算出します
	*   src_cmp = src[0] + src[1] i において
	*   tmp_dst = pow(src[0],2.0) + pow(src[1],2.0)
	*   dst[0] = pow(tmp_dst,0.5*power);
	*
	* \~english
	* @brief Get the norm of the complex signal.
	* @param dst pointer to the output data, of float format, and of length src_n
	* @param src pointer to the input data, of float format, an array of length (2 * src_n) (sequence of complex pairs [real part, imaginary part]). (memory layout, using the row-major convention: (src_n, 2))
	* @param src_n length of the input data
	* @param power exponent to apply to the spectrogram (> 0.0). 1.0: amplitude spectrogram
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   Compute the norm of the input data.
	*   For each src_cmp = src[0] + i * src[1],
	*   tmp_dst = pow(src[0],2.0) + pow(src[1],2.0)
	*   dst[0] = pow(tmp_dst,0.5*power);
	*/
	int AILIA_API ailiaAudioComplexNorm(void* dst, const void* src, const int src_n, float power);


	/**
	* \~japanese
	* @brief 実数STFT結果をメル尺度に変換する
	* @param dst 出力データのポインタ、float 型、外側から mel_n, frame_n 順のメモリレイアウト
	* @param src 入力データのポインタ、float 型、外側から freq_n, frame_n 順のメモリレイアウト
	* @param fb_mtrx メルフィルタバンク、float 型、外側から mel_n, freq_n  順のメモリレイアウト
	* @param freq_n 周波数のインデックス数
	* @param frame_n 入力データの時間フレームの数
	* @param mel_n メル周波数のインデックス数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   入力された実数スペクトログラムをメル尺度に変換します
	*   fb_mtrxには ailiaAudioGetFBMatrix() で取得した係数を与える事が出来ます
	*
	* \~english
	* @brief Convert the real output of the STFT to the mel scale.
	* @param dst pointer to the output data, of float format, of length (mel_n * frame_n), and of memory layout (in row-major convention) (mel_n, frame_n).
	* @param src pointer to the input data, of float format, of length (freq_n * frame_n), and of memory layout (in row-major convention) (freq_n, frame_n).
	* @param fb_mtrx the mel filter-bank, of float format, of length (mel_n * freq_n), and of memory layout (in row-major convention) (mel_n, freq_n).
	* @param freq_n number of frequency indices
	* @param frame_n number of time frames in the input data
	* @param mel_n number of mel frequency indices
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   Converts the real spectrogram given in input to the mel scale.
	*   The argument fb_mtrx can take the coefficients outputted by  ailiaAudioGetFBMatrix() .
	*/
	int AILIA_API ailiaAudioConvertToMel(void* dst, const void* src, const void* fb_mtrx, int freq_n, int frame_n, int mel_n);


	/**
	* \~japanese
	* @brief 実数スペクトログラム/メルスペクトログラムの時間フレーム数を調整します。
	* @param dst 出力データのポインタ、freq_n, dst_frame_n 順のメモリレイアウト
	* @param src 入力データのポインタ、freq_n, src_frame_n 順のメモリレイアウト
	* @param freq_n 周波数のインデックス数
	* @param dst_frame_n 出力データの時間フレームの数
	* @param src_frame_n 入力データの時間フレームの数
	* @param pad_data パディング（dst_frame_n > src_frame_n の場合に使用）
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   dst_frame_n > src_frame_n : 不足する時間フレームのデータを pad_data のデータで埋める。
	*   dst_frame_n <= src_frame_n : 先頭から dst_frame_n のデータのみを切り出す。
	*
	* \~english
	* @brief Fix the number of time frames of a real-valued spectrogram/mel-spectrogram.
	* @param dst pointer to the output data, of length (freq_n * dst_frame_n), and of memory layout (in row-major convention) (freq_n, dst_frame_n).
	* @param src pointer to the input data, of length (freq_n * src_frame_n), and of memory layout (in row-major convention) (freq_n, src_frame_n).
	* @param freq_n number of frequency indices
	* @param dst_frame_n number of time frames in the output data
	* @param src_frame_n number of time frames in the input data
	* @param pad_data value inserted for padding (used when dst_frame_n > src_frame_n)
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   dst_frame_n > src_frame_n : missing time frames are added and filled with the value pad_data.
	*   dst_frame_n <= src_frame_n : only keeps the first dst_frame_n data.
	*/
	int AILIA_API ailiaAudioFixFrameLen(void* dst, const void* src, int freq_n, int dst_frame_n, int src_frame_n, float pad_data);


	/**
	* \~japanese
	* @brief 信号をリサンプルします
	* @param dst 出力データのポインタ、float 型、要素数 dst_n
	* @param src 入力データのポインタ、float 型、要素数 src_n
	* @param dst_sample_rate 変換後のサンプリングレート
	* @param dst_n データ出力先の確保要素数（dst_n >= max_resample_n）
	* @param src_sample_rate 入力データのサンプリングレート
	* @param src_n 入力データの要素数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   最大出力数max_resample_nは ailiaAudioGetResampleLen() で取得できます。
	*   dst_n <  max_resample_n : 先頭からdst_nに入る部分のみ出力
	*   dst_n >= max_resample_n : 出力要素数はmax_resample_n
	*
	* \~english
	* @brief Resample the signal.
	* @param dst pointer to the output data, of float format, and of length dst_n
	* @param src pointer to the input data, of float format, and of length src_n
	* @param dst_sample_rate sampling rate after the resampling
	* @param dst_n length (in number of samples) reserved in the output buffer（dst_n >= max_resample_n）
	* @param src_sample_rate sampling rate of the input signal
	* @param src_n number of samples in the input signal
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   The max number of samples in the output, max_resample_n, can be obtained from  ailiaAudioGetResampleLen() .
	*   dst_n <  max_resample_n : only the first dst_n samples are outputted
	*   dst_n >= max_resample_n : max_resample_n samples are outputted
	*/
	int AILIA_API ailiaAudioResample(void* dst, const void* src, int dst_sample_rate, int dst_n, int src_sample_rate, int src_n);


	/**
	* \~japanese
	* @brief リサンプル後のサンプル数を計算します
	* @param dst_sample_n リサンプル後サンプル数出力先ポインタ
	* @param dst_sample_rate 変換後のサンプリングレート
	* @param src_sample_n 入力データのサンプル数
	* @param src_sample_rate 入力データのサンプリングレート
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	*
	* \~english
	* @brief Get the number of samples after the resampling.
	* @param dst_sample_n pointer to the destination where to write the output (the number of samples after resampling)
	* @param dst_sample_rate sampling rate after the resampling
	* @param src_sample_n number of samples in the input signal
	* @param src_sample_rate sampling rate of the input signal
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	*/
	int AILIA_API ailiaAudioGetResampleLen(int* dst_sample_n, int dst_sample_rate, int src_sample_n, int src_sample_rate);


	/**
	* \~japanese
	* @brief 信号にフィルタ処理を適用します
	* @param dst 出力データのポインタ、float 型、要素数 dst_n
	* @param src 入力データのポインタ、float 型、要素数 src_n
	* @param n_coef フィルタ分子係数のポインタ、float 型、要素数 n_coef_n
	* @param d_coef フィルタ分母係数のポインタ、float 型、要素数 d_coef_n
	* @param zi 遅延状態のポインタ、float 型、要素数 zi_n (zi_n = max(n_coef_n,d_coef_n)-1)、nullptrを許容
	* @param dst_n データ出力先の確保要素数（dst_n >= src_n）
	* @param src_n 入力データの要素数
	* @param n_coef_n フィルタ分子係数の要素数
	* @param d_coef_n フィルタ分母係数の要素数
	* @param zi_n 遅延状態の要素数 (zi_n >= max(n_coef_n,d_coef_n)-1)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   dstへの出力数はmin(dst_m,src_n)となります。
	*   ziへは初期遅延状態を渡します。処理後には最終遅延状態に上書きされます。
	*   zi_nはmax(n_coef_n,d_coef_n)-1が必要となります。不足の場合、不足分は0でパディングし、最終遅延状態は返しません。
	*   ziにnullptrを与えた場合は、初期遅延状態を0とします。最終遅延状態も返しません。zi_nは無視されます。
	*   n_coef_nとd_coef_nは大きいほうを基準とし、不足分は0でパディングします。
	*
	* \~english
	* @brief Apply a filter to the signal.
	* @param dst pointer to the output data, of float format, and of length dst_n
	* @param src pointer to the input data, of float format, and of length src_n
	* @param n_coef pointer to the numerator coefficients of the filter, of float format, and length n_coef_n
	* @param d_coef pointer to the denominator coefficients of the filter, of float format, and length d_coef_n
	* @param zi pointer to the initial delayed values to be used, of float format, and of length zi_n (zi_n = max(n_coef_n,d_coef_n)-1). nullptr is allowed.
	* @param dst_n size, in number of samples, reserved in the output buffer (dst_n >= src_n)
	* @param src_n number of samples in the input signal
	* @param n_coef_n number of numerator coefficients of the filter
	* @param d_coef_n number of denominator coefficients of the filter
	* @param zi_n number of initial delayed values provided (zi_n >= max(n_coef_n,d_coef_n)-1)
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   The number of samples outputted to dst is min(dst_m,src_n).
	*   Use zi to provide the initial delayed values. During processing, this array is overriden with the new delayed values.
	*   Out of the zi_n, the number of delayed values used is max(n_coef_n,d_coef_n)-1. If there are less than that, the remaining is assumed to be zeros, and the array zi is not updated with the new values.
	*   When zi is nullptr, zi_n is ignored, all the delayed values are assumed to be zero, and the new delayed values are not returned.
	*   The largest of n_coef_n and d_coef_n is taken as reference and zeros are added for padding where necessary.
	*/
	int AILIA_API ailiaAudioLinerFilter(void* dst, const void* src, const void* n_coef, const void* d_coef, void* zi, int dst_n, int src_n, int n_coef_n, int d_coef_n, int zi_n);


	/**
	* \~japanese
	* @brief フィルタ処理用の初期遅延係数を算出します
	* @param dst_zi 出力する初期遅延状態のポインタ、float 型、要素数 dst_n (dst_n >= max(n_coef_n,d_coef_n)-1)
	* @param n_coef フィルタ分子係数のポインタ、float 型、要素数 n_coef_n
	* @param d_coef フィルタ分母係数のポインタ、float 型、要素数 d_coef_n
	* @param dst_n 出力先の確保要素数 (dst_n >= max(n_coef_n,d_coef_n)-1)
	* @param n_coef_n フィルタ分子係数の要素数
	* @param d_coef_n フィルタ分母係数の要素数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   一般に、得られた係数に入力信号の先頭を乗じたものを、初期遅延状態として ailiaAudioLinerFilter() に与えます。
	*   dst_nはmax(n_coef_n,d_coef_n)-1が必要となります。
	*   不足の場合は、確保分だけ出力します。
	*   超える部分は、0で埋めます。
	*   n_coef_nとd_coef_nは大きいほうを基準とし、不足分は0でパディングします。
	*
	* \~english
	* @brief Calculate the initial delay coefficients for filtering
	* @param dst_zi pointer to the output (initial delay coefficients), of float format, and of length dst_n (dst_n >= max(n_coef_n,d_coef_n)-1)
	* @param n_coef pointer to the numerator coefficients of the filter, of float format, and length n_coef_n
	* @param d_coef pointer to the denominator coefficients of the filter, of float format, and length d_coef_n
	* @param dst_n size, in number of samples, reserved in the output buffer (dst_n >= max(n_coef_n,d_coef_n)-1)
	* @param n_coef_n number of numerator coefficients of the filter
	* @param d_coef_n number of denominator coefficients of the filter
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   These initial delay coefficients dst_zi, once multiplied with the early values of the signal, can be passed as initial delayed values, the zi argument, to  ailiaAudioLinerFilter() .
	*   Of the dst_n reserved length of the output buffer, the length used is max(n_coef_n,d_coef_n)-1.
	*   If dst_n is less than that, only the corresponding first values are output.
	*   If dst_n is larger, the remaining is filled with 0.
	*   The largest of n_coef_n and d_coef_n is taken as reference and zeros are added for padding where necessary.
	*/
	int AILIA_API ailiaAudioGetLinerFilterZiCoef(void* dst_zi, const void* n_coef, const void* d_coef, int dst_n, int n_coef_n, int d_coef_n);


	/**
	* \~japanese
	* @brief 信号にゼロ位相フィルタ処理を適用します
	* @param dst 出力データのポインタ、float 型、要素数 dst_n
	* @param src 入力データのポインタ、float 型、要素数 src_n
	* @param n_coef フィルタ分子係数のポインタ、float 型、要素数 n_coef_n
	* @param d_coef フィルタ分母係数のポインタ、float 型、要素数 d_coef_n
	* @param dst_n データ出力先の確保要素数（dst_n >= src_n）
	* @param src_n 入力データの要素数
	* @param n_coef_n フィルタ分子係数の要素数
	* @param d_coef_n フィルタ分母係数の要素数
	* @param pad_type 入力信号に対する両端パディング処理方法、	AILIA_AUDIO_FILTFILT_PAD_* のいずれか
	* @param pad_len 入力信号に対する両端パディング数
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   dstへの出力数はmin(dst_m,src_n)となります。
	*   n_coef_nとd_coef_nは大きいほうを基準とし、不足分は0でパディングします。
	*
	* \~english
	* @brief Apply a zero-phase filter to the signal.
	* @param dst pointer to the output data, of float format, and of length dst_n
	* @param src pointer to the input data, of float format, and of length src_n
	* @param n_coef pointer to the numerator coefficients of the filter, of float format, and length n_coef_n
	* @param d_coef pointer to the denominator coefficients of the filter, of float format, and length d_coef_n
	* @param dst_n length (in number of samples) reserved in the output buffer (dst_n >= src_n)
	* @param src_n number of samples in the input signal
	* @param n_coef_n number of numerator coefficients of the filter
	* @param d_coef_n number of denominator coefficients of the filter
	* @param pad_type type of padding to apply at the start and at the end of the input signal: any of the AILIA_AUDIO_FILTFILT_PAD_* constants
	* @param pad_len length of the padding applied to the start and to the end of the input signal
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   The number of values written to the output dst is min(dst_m,src_n).
	*   The largest of n_coef_n and d_coef_n is taken as reference and zeros are added for padding where necessary.
	*/
	int AILIA_API ailiaAudioFilterFilter(void* dst, const void* src, const void* n_coef, const void* d_coef, int dst_n, int src_n, int n_coef_n, int d_coef_n, int pad_type, int pad_len);


	/**
	* \~japanese
	* @brief 信号の入力前後の無音域を除いた領域を検出します
	* @param dst_start_pos 有音域の先頭サンプル位置出力先ポインタ、int 型
	* @param dst_length 有音域の長さ出力先ポインタ、int 型
	* @param src 入力データのポインタ、float 型、要素数 sample_n
	* @param sample_n 入力データのサンプル数
	* @param win_n 1フレームに含むサンプル数
	* @param hop_n フレームのシフト数
	* @param thr_db 有音を判断するdB (thr_db > 0)
	* @return
	*   成功した場合は \ref AILIA_STATUS_SUCCESS 、そうでなければエラーコードを返す。
	* @details
	*   全域が無音の場合、*dst_start_pos = -1,*dst_length = 0となります。
	*
	* \~english
	* @brief Find the region of the signal between the first and the last non-silence samples. Detects the area excluding the silent range before and after the signal input
	* @param dst_start_pos pointer to the destination where to write the outputted start position of the non-silence area, of int format
	* @param dst_length pointer to the destination where to write the outputted length of the non-silence area, of int format
	* @param src pointer to the input data, of float format, and of length sample_n
	* @param sample_n count of samples in the input data
	* @param win_n size of the window function
	* @param hop_n stride of each window shift (in number of samples)
	* @param thr_db threshold (in dB) above which the signal is considered non-silence (thr_db > 0)
	* @return
	*   In case of success,  \ref AILIA_STATUS_SUCCESS , and else an error code is returned.
	* @details
	*   In case the whole signal is considered silence, the following happens: *dst_start_pos = -1, *dst_length = 0
	*/
	int AILIA_API ailiaAudioGetNonSilentPos(int* dst_start_pos, int* dst_length, const void* src, int sample_n, int win_n, int hop_n, float thr_db);


#ifdef __cplusplus
}
#endif
#endif /* !defined(INCLUDED_AILIA_AUDIO) */
