cmd_/home/pi/mpu6050/mpu6050_custom.ko := ld -EL -r  -T ./scripts/module-common.lds --build-id  -o /home/pi/mpu6050/mpu6050_custom.ko /home/pi/mpu6050/mpu6050_custom.o /home/pi/mpu6050/mpu6050_custom.mod.o