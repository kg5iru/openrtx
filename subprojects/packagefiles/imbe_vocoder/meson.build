project('imbe_vocoder', 'cpp')

# IMBE 4400 bps vocoder for P25 Phase 1 voice: the imbe_embedded fixed-point
# engine (GPL-3.0-or-later) with its imbe2 C API, built as a static library
# the same way codec2 is.

imbe_vocoder_inc = include_directories('inc')
imbe_vocoder_src = ['src/aux_sub.cc',
                    'src/basicop2.cc',
                    'src/ch_decode.cc',
                    'src/ch_encode.cc',
                    'src/dc_rmv.cc',
                    'src/decode.cc',
                    'src/dsp_sub.cc',
                    'src/encode.cc',
                    'src/imbe2_api.cc',
                    'src/imbe_vocoder.cc',
                    'src/math_sub.cc',
                    'src/pe_lpf.cc',
                    'src/pitch_est.cc',
                    'src/pitch_ref.cc',
                    'src/qnt_sub.cc',
                    'src/rand_gen.cc',
                    'src/sa_decode.cc',
                    'src/sa_encode.cc',
                    'src/sa_enh.cc',
                    'src/tbls.cc',
                    'src/uv_synt.cc',
                    'src/v_synt.cc',
                    'src/v_uv_det.cc']

add_project_arguments('-w', language : 'cpp')

imbe_vocoder = static_library('imbe_vocoder',
                              imbe_vocoder_src,
                              include_directories : imbe_vocoder_inc,
                              install : false)

imbe_vocoder_dep = declare_dependency(include_directories : imbe_vocoder_inc,
                                      link_with : imbe_vocoder)
