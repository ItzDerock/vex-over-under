#pragma once

class ExitCondition {
 public:
  /**
   * @brief Create a new Exit Condition
   *
   * @param range the range where the countdown is allowed to start
   * @param time how much time to wait while in range before exiting
   */
  ExitCondition(const float range, const int time);

  /**
   * @brief whether the exit condition has been met
   *
   * @return true exit condition met
   * @return false exit condition not met
   */
  bool getExit() const;

  /**
   * @brief update the exit condition
   *
   * @param input the input for the exit condition
   * @return true exit condition met
   * @return false exit condition not met
   */
  bool update(const float input);

  /**
   * @brief reset the exit condition timer
   *
   */
  void reset();

  /**
   * @brief sets the exit conditions
   *
   * @param range the range where the countdown is allowed to start
   * @param time how much time to wait while in range before exiting
   */
  void setExit(const float range, const int time);

 private:
  float range;
  int time;
  int startTime = -1;
  bool done = false;
};