/* ========== use corner for optical flow ========== */
Eigen::Vector2f opticalFlowCorner(cv::Mat image)
{
  cv::Mat gray, draw;
  image.copyTo(gray);
  cv::cvtColor(image, draw, cv::COLOR_GRAY2BGR);
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
  cv::Size sub_pix_win_size(10, 10), win_size(31, 31);

  /* ---------- initialize ---------- */
  if (!init)
  {
    /*  init with corner features  */
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(gray, corners, 500, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
    cv::cornerSubPix(gray, corners, sub_pix_win_size, cv::Size(-1, -1), termcrit);
    std::swap(prev_pts, corners);

    init = true;
  }
  else
  {
    /* ---------- opencv calc optical flow ---------- */
    vector<uchar> status;
    vector<float> err;
    vector<cv::Point2f> next_pts;

    cv::calcOpticalFlowPyrLK(prev_gray, gray, prev_pts, next_pts, status, err, win_size, 3, termcrit, 0,
                             0.001);

    /* ---------- draw result ---------- */
    for (int i = 0; i < next_pts.size(); i++)
    {
      if (status[i] == 0) continue;

      cv::circle(draw, next_pts[i], 3, cv::Scalar(255, 0, 0), -1, 8);
      cv::line(draw, next_pts[i], prev_pts[i], cv::Scalar(0, 0, 255), 1, 8);
    }

    std::swap(prev_pts, next_pts);
  }

  std::swap(prev_gray, gray);
  cv::imshow("LK", draw);

  return Eigen::Vector2f();
}
