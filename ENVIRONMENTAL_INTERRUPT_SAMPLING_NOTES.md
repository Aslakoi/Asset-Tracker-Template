# Environmental Interrupt-Based Sampling Implementation Notes

## Debug Output

✅ **Debug prints with actual sensor data are included:**

1. **periodic_sample_work_handler()** - Logs raw sensor readings:
   - BMI270: `LOG_DBG("BMI270 sampled: accel_hp[%.3f, %.3f, %.3f] g, gyro_hp[%.3f, %.3f, %.3f] dps", ...)`
   - ADXL367: `LOG_DBG("ADXL367 sampled: accel_lp[%.3f, %.3f, %.3f] g", ...)`

2. **fifo_process_work_handler()** - Logs messages being published:
   - `LOG_DBG("Publishing sensor sample: accel_hp[%.2f, %.2f, %.2f] g, gyro_hp[%.2f, %.2f, %.2f] dps, accel_lp[%.2f, %.2f, %.2f] g", ...)`

3. **Trigger events:**
   - `LOG_DBG("Sensor data-ready trigger from %s, submitting sample work", sensor->name)`

## Zephyr ISR Offloading Pattern - Compliance

✅ **Implementation follows Zephyr's recommended ISR offloading pattern:**

The original code called `periodic_sample_work_handler(NULL)` directly in the interrupt handler. This has been **corrected**:

```c
// BEFORE (Direct handler call - NOT recommended)
static void sensor_trigger_callback(...) {
    periodic_sample_work_handler(NULL);  // ✗ Blocking ISR
}

// AFTER (Work queue offloading - Recommended)
static void sensor_trigger_callback(...) {
    k_work_submit(&periodic_sample_work);  // ✓ Offloads to workqueue
}
```

**Why this matters:**
- ISRs should be short and non-blocking
- Heavy work (sensor I/O, calculations) must offload to a thread/workqueue
- `k_work_submit()` signals the work but returns immediately
- The actual sampling happens in a dedicate worker thread/workqueue context

## Ring Buffer vs. K_FIFO - Design Choice

**Why `ring_buf` instead of `K_FIFO`:**

| Aspect | ring_buf | K_FIFO |
|--------|----------|--------|
| Memory allocation | Static (stack) | Dynamic (heap) |
| Overhead | Minimal | Requires CONFIG_HEAP_MEM_POOL_SIZE |
| Fixed-size items | ✓ Efficient | ✓ Works but less efficient |
| Pattern | Polling/delayable work | ISR-driven signaling |
| Flexibility | Can hold multiple samples before processing | Typically one item at a time |

**Trade-off made:** Using `ring_buf` with delayable work is appropriate here because:
1. **Predictable sampling period** - 50 Hz fixed periodic sampling
2. **Batching efficiency** - Multiple samples can be buffered before zbus publication
3. **Memory efficiency** - Static allocation, no malloc overhead
4. **ISR optional** - Triggers are optional; polling is the primary mechanism

**Alternative: K_FIFO Pattern**
If you want traditional ISR-driven with K_FIFO (preferred for edge-triggered interrupts):
```c
#define K_FIFO_DEFINE(sensor_fifo);

struct sensor_data_item {
    void *fifo_reserved;  // Required by Zephyr FIFO
    double accel_hp[3];
    double gyro_hp[3];
    double accel_lp[3];
    int64_t timestamp;
};
```

## Hardware Context - What's Verified

⚠️ **Important Hardware Notes:**

The Thingy:91 X board is NOT in the provided knowledge sources. The following sensors are confirmed on different boards:

| Sensor | Board | Notes |
|--------|-------|-------|
| BMI270 | Thingy:53 (nRF5340) | Confirmed accelerometer + gyroscope |
| ADXL367 | nRF54L15 Tag | Confirmed accelerometer (I2C) |
| BME680 | Thingy:53 | Confirmed environmental sensor |

**What's NOT confirmed from knowledge sources:**
- ADXL367 presence on Thingy:91 X (may be ADXL362 instead)
- Exact I2C/SPI pin mappings on Thingy:91 X
- Interrupt pin assignments (P0.06, P0.11, etc.)

**Recommendation:**
Verify with your device tree overlay and hardware datasheet:
```bash
grep -r "bmi270\|adxl367\|adxl362" app/boards/thingy91x_nrf9151_ns.*
```

## Integration with Storage Module

Data flows correctly to storage module via zbus:
1. **Source:** environmental_chan (ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE)
2. **Storage behavior:** Handled by storage_data_types.c:
   ```c
   bool environmental_check(const struct environmental_msg *msg) {
       return msg->type == ENVIRONMENTAL_SENSOR_SAMPLE_RESPONSE;
   }
   ```
3. **Result:** Each sample published to environmental_chan is automatically captured by storage module

## Verification Checklist

- [x] Debug output includes actual sensor values
- [x] ISR offloading pattern corrected (k_work_submit)
- [x] FIFO/ring_buf choice documented
- [ ] **TODO:** Verify BMI270 and ADXL367 are present on your Thingy:91 X
- [ ] **TODO:** Verify interrupt pins and device tree nodes
- [ ] **TODO:** Build and test compilation

## Next Steps

1. **Verify device tree** - Check `thingy91x_nrf9151_ns.overlay` for sensor definitions
2. **Check kernel config** - Ensure `CONFIG_HEAP_MEM_POOL_SIZE` is sufficient (minimum 4096 for sampling)
3. **Test build:**
   ```bash
   west build -b thingy91x_nrf9151_ns app/
   ```
4. **Monitor logs** - Enable debug logging to see actual sensor data flowing
