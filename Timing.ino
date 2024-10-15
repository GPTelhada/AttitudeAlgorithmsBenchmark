// Global variable to store the start time for the timer
uint32_t startTimer;

// Function to start the timer by capturing the current microsecond count
void start() {
  startTimer = micros();  // Record the current time in microseconds
}

// Function to stop the timer and return the elapsed time in microseconds
uint32_t stop() {
  return micros() - startTimer;  // Calculate and return the elapsed time
}
