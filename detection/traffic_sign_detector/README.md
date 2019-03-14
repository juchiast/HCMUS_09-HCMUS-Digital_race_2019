# README Traffic Sign Detector

## Mục tiêu
Nhận dạng biển báo gồm 2 loại [trái, phải]

## Input
- RGB Image. Có thể là một bounding box để recognize (từ object_detector package) hoặc bao gồm cả detection và recognization

## Output
- Thông tin về biển báo (xem cụ thể định nghĩa trong  `cds_msgs/msg/SignDetected.msg`)
  - Timestamp: Thời điểm nhận được frame hình (trong std_msgs/Header header)
  - id:
    - `0`: `SLOW`
    - `1`: `LEFT`
    - `2`: `RIGHT`
    - `-1`: `NONE`
  - Bounding box:
    - `x`, `y`, `width`, `height`

## Topic Subscribe
- Trực tiếp từ `/camera/rgb/image_raw` nếu detection + recognition
- Từ `/object_detected_array` nếu kết hợp detection từ `ObjectDetector`

## Topic Publish
- `/sign_detected` với message type `cds_msgs/SignDetected.msg`