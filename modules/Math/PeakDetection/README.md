# Peak Detection {#peak_detection}

PeakDetection folder includes two algorithms:

CA CFAR (Cell Averaging Constant False Alarm Rate)
  https://en.wikipedia.org/wiki/Constant_false_alarm_rate
  This is adaptive type of peak detection. It takes in account neighbor cells to detect a peak based on preset False Alarm rate. Returns location of complying peaks.

Gradient Peak Detection
  Detects peak by gradient (slope) distribution in vector. When slope changes sign, it identifies it as a peak. Returns location of positive peaks.
