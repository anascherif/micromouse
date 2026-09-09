/**
 * @file TaskBase.h
 * @brief FreeRTOS task wrapper for C++ classes.
 * Ported from kerise-v3.
 */
#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

class TaskBase {
public:
  TaskBase() : task_handle_(nullptr) {}
  virtual ~TaskBase() { deleteTask(); }

  bool createTask(const char* name, UBaseType_t priority = 0,
                  uint16_t stack_depth = configMINIMAL_STACK_SIZE,
                  BaseType_t core_id = tskNO_AFFINITY) {
    if (task_handle_ != nullptr) return false;
    BaseType_t res = xTaskCreatePinnedToCore(
        taskEntry, name, stack_depth, this, priority, &task_handle_, core_id);
    return res == pdPASS;
  }

  void deleteTask() {
    if (task_handle_ != nullptr) {
      vTaskDelete(task_handle_);
      task_handle_ = nullptr;
    }
  }

protected:
  TaskHandle_t task_handle_;

  virtual void task() = 0;

private:
  static void taskEntry(void* pvParameters) {
    static_cast<TaskBase*>(pvParameters)->task();
  }
};