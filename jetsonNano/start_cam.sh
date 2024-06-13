#gst-launch-1.0 zedsrc ! autovideoconvert ! \
# x264enc byte-stream=true tune=zerolatency speed-preset=ultrafast bitrate=9000 ! \
# h264parse ! rtph264pay config-interval=-1 pt=96 ! queue ! \
# udpsink clients=10.42.0.45:5000 max-bitrate=3000000 sync=false async=false
gst-launch-1.0 zedsrc ! videoconvert ! x264enc tune=zerolatency speed-preset=ultrafast ! rtph264pay ! udpsink clients=10.42.0.45:5000 max-bitrate=9000000 sync=false async=true
