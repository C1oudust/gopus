// +build !amd64,!386,cgo

package gopus

// #cgo !nopkgconfig pkg-config: opus
//
// #include <opus.h>
// enum {
//   gopus_ok = OPUS_OK,
//   gopus_bad_arg = OPUS_BAD_ARG,
//   gopus_small_buffer = OPUS_BUFFER_TOO_SMALL,
//   gopus_internal = OPUS_INTERNAL_ERROR,
//   gopus_invalid_packet = OPUS_INVALID_PACKET,
//   gopus_unimplemented = OPUS_UNIMPLEMENTED,
//   gopus_invalid_state = OPUS_INVALID_STATE,
//   gopus_alloc_fail = OPUS_ALLOC_FAIL,
// };
//
//
// enum {
//   gopus_application_voip    = OPUS_APPLICATION_VOIP,
//   gopus_application_audio   = OPUS_APPLICATION_AUDIO,
//   gopus_restricted_lowdelay = OPUS_APPLICATION_RESTRICTED_LOWDELAY,
//   gopus_bitrate_max         = OPUS_BITRATE_MAX,
// };
//
//
// void gopus_setvbr(OpusEncoder *encoder, int vbr) {
//   opus_encoder_ctl(encoder, OPUS_SET_VBR(vbr));
// }
//
// void gopus_setbitrate(OpusEncoder *encoder, int bitrate) {
//   opus_encoder_ctl(encoder, OPUS_SET_BITRATE(bitrate));
// }
//
// opus_int32 gopus_bitrate(OpusEncoder *encoder) {
//   opus_int32 bitrate;
//   opus_encoder_ctl(encoder, OPUS_GET_BITRATE(&bitrate));
//   return bitrate;
// }
//
// void gopus_setapplication(OpusEncoder *encoder, int application) {
//   opus_encoder_ctl(encoder, OPUS_SET_APPLICATION(application));
// }
//
// opus_int32 gopus_application(OpusEncoder *encoder) {
//   opus_int32 application;
//   opus_encoder_ctl(encoder, OPUS_GET_APPLICATION(&application));
//   return application;
// }
//
// void gopus_encoder_resetstate(OpusEncoder *encoder) {
//   opus_encoder_ctl(encoder, OPUS_RESET_STATE);
// }
//
// void gopus_decoder_resetstate(OpusDecoder *decoder) {
//   opus_decoder_ctl(decoder, OPUS_RESET_STATE);
// }
//
// opus_int32 gopus_setfec(OpusEncoder *encoder, int fec) {
//   return opus_encoder_ctl(encoder, OPUS_SET_INBAND_FEC(fec));
// }
//
// opus_int32 gopus_setdtx(OpusEncoder *encoder, int dtx) {
//   return opus_encoder_ctl(encoder, OPUS_SET_DTX(dtx));
// }
//
// void gopus_set_complexity(OpusEncoder *encoder, int complexity) {
//   opus_encoder_ctl(encoder, OPUS_SET_COMPLEXITY(complexity));
// }
//
// opus_int32 gopus_get_complexity(OpusEncoder *encoder) {
//   opus_int32 complexity;
//   opus_encoder_ctl(encoder, OPUS_GET_COMPLEXITY(&complexity));
//   return complexity;
// }
//
// void gopus_set_max_bandwidth(OpusEncoder *encoder, int bandwidth) {
//   opus_encoder_ctl(encoder, OPUS_SET_MAX_BANDWIDTH(bandwidth));
// }
//
// opus_int32 gopus_get_max_bandwidth(OpusEncoder *encoder) {
//   opus_int32 bandwidth;
//   opus_encoder_ctl(encoder, OPUS_GET_MAX_BANDWIDTH(&bandwidth));
//   return bandwidth;
// }
//
// void gopus_set_bandwidth(OpusEncoder *encoder, int bandwidth) {
//   opus_encoder_ctl(encoder, OPUS_SET_BANDWIDTH(bandwidth));
// }
//
// void gopus_set_signal(OpusEncoder *encoder, int signal) {
//   opus_encoder_ctl(encoder, OPUS_SET_SIGNAL(signal));
// }
//
// opus_int32 gopus_get_signal(OpusEncoder *encoder) {
//   opus_int32 signal;
//   opus_encoder_ctl(encoder, OPUS_GET_SIGNAL(&signal));
//   return signal;
// }
//
// void gopus_set_vbr_constraint(OpusEncoder *encoder, int constraint) {
//   opus_encoder_ctl(encoder, OPUS_SET_VBR_CONSTRAINT(constraint));
// }
//
// opus_int32 gopus_get_vbr_constraint(OpusEncoder *encoder) {
//   opus_int32 constraint;
//   opus_encoder_ctl(encoder, OPUS_GET_VBR_CONSTRAINT(&constraint));
//   return constraint;
// }
//
// void gopus_set_force_channels(OpusEncoder *encoder, int channels) {
//   opus_encoder_ctl(encoder, OPUS_SET_FORCE_CHANNELS(channels));
// }
//
// opus_int32 gopus_get_force_channels(OpusEncoder *encoder) {
//   opus_int32 channels;
//   opus_encoder_ctl(encoder, OPUS_GET_FORCE_CHANNELS(&channels));
//   return channels;
// }
//
// void gopus_set_packet_loss_perc(OpusEncoder *encoder, int perc) {
//   opus_encoder_ctl(encoder, OPUS_SET_PACKET_LOSS_PERC(perc));
// }
//
// opus_int32 gopus_get_packet_loss_perc(OpusEncoder *encoder) {
//   opus_int32 perc;
//   opus_encoder_ctl(encoder, OPUS_GET_PACKET_LOSS_PERC(&perc));
//   return perc;
// }
//
// void gopus_set_lsb_depth(OpusEncoder *encoder, int depth) {
//   opus_encoder_ctl(encoder, OPUS_SET_LSB_DEPTH(depth));
// }
//
// opus_int32 gopus_get_lsb_depth(OpusEncoder *encoder) {
//   opus_int32 depth;
//   opus_encoder_ctl(encoder, OPUS_GET_LSB_DEPTH(&depth));
//   return depth;
// }
//
// void gopus_set_prediction_disabled(OpusEncoder *encoder, int disabled) {
//   opus_encoder_ctl(encoder, OPUS_SET_PREDICTION_DISABLED(disabled));
// }
//
// opus_int32 gopus_get_prediction_disabled(OpusEncoder *encoder) {
//   opus_int32 disabled;
//   opus_encoder_ctl(encoder, OPUS_GET_PREDICTION_DISABLED(&disabled));
//   return disabled;
// }
//
// void gopus_set_phase_inversion_disabled(OpusEncoder *encoder, int disabled) {
//   opus_encoder_ctl(encoder, OPUS_SET_PHASE_INVERSION_DISABLED(disabled));
// }
//
// opus_int32 gopus_get_phase_inversion_disabled(OpusEncoder *encoder) {
//   opus_int32 disabled;
//   opus_encoder_ctl(encoder, OPUS_GET_PHASE_INVERSION_DISABLED(&disabled));
//   return disabled;
// }
//
// void gopus_set_dred_duration(OpusEncoder *encoder, int duration) {
//   opus_encoder_ctl(encoder, OPUS_SET_DRED_DURATION(duration));
// }
//
// opus_int32 gopus_get_dred_duration(OpusEncoder *encoder) {
//   opus_int32 duration;
//   opus_encoder_ctl(encoder, OPUS_GET_DRED_DURATION(&duration));
//   return duration;
// }
import "C"

import (
	"errors"
	"unsafe"
)

type Application int

const (
	Voip               Application = C.gopus_application_voip
	Audio              Application = C.gopus_application_audio
	RestrictedLowDelay Application = C.gopus_restricted_lowdelay
)

type Signal int
type Bandwidth int

const (
	// Auto bandwidth or signal type.
	Auto Signal = C.OPUS_AUTO
	// Voice signal type.
	Voice Signal = C.OPUS_SIGNAL_VOICE
	// Music signal type.
	Music Signal = C.OPUS_SIGNAL_MUSIC
)

const (
	// BandwidthAuto lets the encoder decide the bandwidth.
	BandwidthAuto Bandwidth = C.OPUS_AUTO
	// Narrowband is 4kHz bandpass.
	Narrowband Bandwidth = C.OPUS_BANDWIDTH_NARROWBAND
	// Mediumband is 6kHz bandpass.
	Mediumband Bandwidth = C.OPUS_BANDWIDTH_MEDIUMBAND
	// Wideband is 8kHz bandpass.
	Wideband Bandwidth = C.OPUS_BANDWIDTH_WIDEBAND
	// Superwideband is 12kHz bandpass.
	Superwideband Bandwidth = C.OPUS_BANDWIDTH_SUPERWIDEBAND
	// Fullband is 20kHz bandpass.
	Fullband Bandwidth = C.OPUS_BANDWIDTH_FULLBAND
)

const (
	// ChannelsAuto lets the encoder decide the number of channels.
	ChannelsAuto = C.OPUS_AUTO
)

const (
	BitrateMaximum = C.gopus_bitrate_max
)

type Encoder struct {
	data     []byte
	cEncoder *C.struct_OpusEncoder
}

func NewEncoder(sampleRate, channels int, application Application) (*Encoder, error) {
	encoder := &Encoder{}
	encoder.data = make([]byte, int(C.opus_encoder_get_size(C.int(channels))))
	encoder.cEncoder = (*C.struct_OpusEncoder)(unsafe.Pointer(&encoder.data[0]))

	ret := C.opus_encoder_init(encoder.cEncoder, C.opus_int32(sampleRate), C.int(channels), C.int(application))
	if err := getErr(ret); err != nil {
		return nil, err
	}
	return encoder, nil
}

func (e *Encoder) Encode(pcm []int16, frameSize, maxDataBytes int) ([]byte, error) {
	pcmPtr := (*C.opus_int16)(unsafe.Pointer(&pcm[0]))

	data := make([]byte, maxDataBytes)
	dataPtr := (*C.uchar)(unsafe.Pointer(&data[0]))

	encodedC := C.opus_encode(e.cEncoder, pcmPtr, C.int(frameSize), dataPtr, C.opus_int32(len(data)))
	encoded := int(encodedC)

	if encoded < 0 {
		return nil, getErr(C.int(encodedC))
	}
	return data[0:encoded], nil
}

func (e *Encoder) SetVbr(vbr bool) {
	var cVbr C.int
	if vbr {
		cVbr = 1
	} else {
		cVbr = 0
	}
	C.gopus_setvbr(e.cEncoder, cVbr)
}

func (e *Encoder) SetBitrate(bitrate int) {
	C.gopus_setbitrate(e.cEncoder, C.int(bitrate))
}

func (e *Encoder) Bitrate() int {
	return int(C.gopus_bitrate(e.cEncoder))
}

func (e *Encoder) SetApplication(application Application) {
	C.gopus_setapplication(e.cEncoder, C.int(application))
}

func (e *Encoder) Application() Application {
	return Application(C.gopus_application(e.cEncoder))
}

func (e *Encoder) ResetState() {
	C.gopus_encoder_resetstate(e.cEncoder)
}

// SetInbandFEC enables or disables Forward Error Correction
func (e *Encoder) SetInbandFEC(enable bool) error {
	var val C.int
	if enable {
		val = 1
	}
	ret := C.gopus_setfec(e.cEncoder, val)
	return getErr(ret)
}

// SetDTX enables or disables Discontinuous Transmission
func (e *Encoder) SetDTX(enable bool) error {
	var val C.int
	if enable {
		val = 1
	}
	ret := C.gopus_setdtx(e.cEncoder, val)
	return getErr(ret)
}

// SetComplexity configures the encoder's computational complexity.
// The supported range is 0-10 inclusive with 10 representing the highest complexity.
func (e *Encoder) SetComplexity(complexity int) {
	C.gopus_set_complexity(e.cEncoder, C.int(complexity))
}

// Complexity gets the encoder's complexity configuration.
// Returns a value in the range 0-10, inclusive.
func (e *Encoder) Complexity() int {
	return int(C.gopus_get_complexity(e.cEncoder))
}

// SetMaxBandwidth configures the maximum bandpass that the encoder will select automatically.
// Applications should normally use this instead of SetBandwidth.
func (e *Encoder) SetMaxBandwidth(bandwidth Bandwidth) {
	C.gopus_set_max_bandwidth(e.cEncoder, C.int(bandwidth))
}

// MaxBandwidth gets the encoder's configured maximum allowed bandpass.
func (e *Encoder) MaxBandwidth() Bandwidth {
	return Bandwidth(C.gopus_get_max_bandwidth(e.cEncoder))
}

// SetBandwidth sets the encoder's bandpass to a specific value.
// This prevents the encoder from automatically selecting the bandpass based on the available bitrate.
func (e *Encoder) SetBandwidth(bandwidth Bandwidth) {
	C.gopus_set_bandwidth(e.cEncoder, C.int(bandwidth))
}

// SetSignal configures the type of signal being encoded.
// This is a hint which helps the encoder's mode selection.
func (e *Encoder) SetSignal(signal Signal) {
	C.gopus_set_signal(e.cEncoder, C.int(signal))
}

// Signal gets the encoder's configured signal type.
func (e *Encoder) Signal() Signal {
	return Signal(C.gopus_get_signal(e.cEncoder))
}

// SetVBRConstraint enables or disables constrained VBR in the encoder.
// This setting is ignored when the encoder is in CBR mode.
func (e *Encoder) SetVBRConstraint(enable bool) {
	var val C.int
	if enable {
		val = 1
	}
	C.gopus_set_vbr_constraint(e.cEncoder, val)
}

// VBRConstraint determines if constrained VBR is enabled in the encoder.
func (e *Encoder) VBRConstraint() bool {
	return C.gopus_get_vbr_constraint(e.cEncoder) != 0
}

// SetForceChannels configures mono/stereo forcing in the encoder.
// Use ChannelsAuto for default behavior.
func (e *Encoder) SetForceChannels(channels int) {
	C.gopus_set_force_channels(e.cEncoder, C.int(channels))
}

// ForceChannels gets the encoder's forced channel configuration.
func (e *Encoder) ForceChannels() int {
	return int(C.gopus_get_force_channels(e.cEncoder))
}

// SetPacketLossPerc configures the encoder's expected packet loss percentage.
// Higher values trigger progressively more loss resistant behavior.
func (e *Encoder) SetPacketLossPerc(perc int) {
	C.gopus_set_packet_loss_perc(e.cEncoder, C.int(perc))
}

// PacketLossPerc gets the encoder's configured packet loss percentage.
func (e *Encoder) PacketLossPerc() int {
	return int(C.gopus_get_packet_loss_perc(e.cEncoder))
}

// SetLSBDepth configures the depth of the signal being encoded.
// It represents the number of significant bits of linear intensity.
func (e *Encoder) SetLSBDepth(depth int) {
	C.gopus_set_lsb_depth(e.cEncoder, C.int(depth))
}

// LSBDepth gets the encoder's configured signal depth.
func (e *Encoder) LSBDepth() int {
	return int(C.gopus_get_lsb_depth(e.cEncoder))
}

// SetPredictionDisabled disables almost all use of prediction, making frames almost completely independent.
// This reduces quality but can be useful for debugging or specific applications.
func (e *Encoder) SetPredictionDisabled(disable bool) {
	var val C.int
	if disable {
		val = 1
	}
	C.gopus_set_prediction_disabled(e.cEncoder, val)
}

// PredictionDisabled gets the encoder's configured prediction status.
func (e *Encoder) PredictionDisabled() bool {
	return C.gopus_get_prediction_disabled(e.cEncoder) != 0
}

// SetPhaseInversionDisabled disables the use of phase inversion for intensity stereo.
// This improves the quality of mono downmixes but slightly reduces normal stereo quality.
func (e *Encoder) SetPhaseInversionDisabled(disable bool) {
	var val C.int
	if disable {
		val = 1
	}
	C.gopus_set_phase_inversion_disabled(e.cEncoder, val)
}

// PhaseInversionDisabled gets the encoder's configured phase inversion status.
func (e *Encoder) PhaseInversionDisabled() bool {
	return C.gopus_get_phase_inversion_disabled(e.cEncoder) != 0
}

// SetDREDDuration enables Deep Redundancy (DRED) and sets the maximum number of 10-ms redundant frames.
// A value of 0 disables DRED.
func (e *Encoder) SetDREDDuration(duration int) {
	C.gopus_set_dred_duration(e.cEncoder, C.int(duration))
}

// DREDDuration gets the encoder's configured Deep Redundancy (DRED) maximum number of frames.
func (e *Encoder) DREDDuration() int {
	return int(C.gopus_get_dred_duration(e.cEncoder))
}

type Decoder struct {
	data     []byte
	cDecoder *C.struct_OpusDecoder
	channels int
}

func NewDecoder(sampleRate, channels int) (*Decoder, error) {
	decoder := &Decoder{}
	decoder.data = make([]byte, int(C.opus_decoder_get_size(C.int(channels))))
	decoder.cDecoder = (*C.struct_OpusDecoder)(unsafe.Pointer(&decoder.data[0]))

	ret := C.opus_decoder_init(decoder.cDecoder, C.opus_int32(sampleRate), C.int(channels))
	if err := getErr(ret); err != nil {
		return nil, err
	}
	decoder.channels = channels

	return decoder, nil
}

func (d *Decoder) Decode(data []byte, frameSize int, fec bool) ([]int16, error) {
	var dataPtr *C.uchar
	if len(data) > 0 {
		dataPtr = (*C.uchar)(unsafe.Pointer(&data[0]))
	}
	dataLen := C.opus_int32(len(data))

	output := make([]int16, d.channels*frameSize)
	outputPtr := (*C.opus_int16)(unsafe.Pointer(&output[0]))

	var cFec C.int
	if fec {
		cFec = 1
	} else {
		cFec = 0
	}

	cRet := C.opus_decode(d.cDecoder, dataPtr, dataLen, outputPtr, C.int(frameSize), cFec)
	ret := int(cRet)

	if ret < 0 {
		return nil, getErr(cRet)
	}
	return output[:ret*d.channels], nil
}

func (d *Decoder) ResetState() {
	C.gopus_decoder_resetstate(d.cDecoder)
}

func CountFrames(data []byte) (int, error) {
	dataPtr := (*C.uchar)(unsafe.Pointer(&data[0]))
	cLen := C.opus_int32(len(data))

	cRet := C.opus_packet_get_nb_frames(dataPtr, cLen)
	if err := getErr(cRet); err != nil {
		return 0, err
	}
	return int(cRet), nil
}

var (
	ErrBadArgument   = errors.New("bad argument")
	ErrSmallBuffer   = errors.New("buffer is too small")
	ErrInternal      = errors.New("internal error")
	ErrInvalidPacket = errors.New("invalid packet")
	ErrUnimplemented = errors.New("unimplemented")
	ErrInvalidState  = errors.New("invalid state")
	ErrAllocFail     = errors.New("allocation failed")
	ErrUnknown       = errors.New("unknown error")
)

func getErr(code C.int) error {
	switch code {
	case C.gopus_ok:
		return nil
	case C.gopus_bad_arg:
		return ErrBadArgument
	case C.gopus_small_buffer:
		return ErrSmallBuffer
	case C.gopus_internal:
		return ErrInternal
	case C.gopus_invalid_packet:
		return ErrInvalidPacket
	case C.gopus_unimplemented:
		return ErrUnimplemented
	case C.gopus_invalid_state:
		return ErrInvalidState
	case C.gopus_alloc_fail:
		return ErrAllocFail
	default:
		return ErrUnknown
	}
}
