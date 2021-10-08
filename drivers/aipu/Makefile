SRC_DIR  := src
FOPS_OBJ := $(SRC_DIR)/aipu_fops.o \
            $(SRC_DIR)/aipu_session.o
MEM_OBJ  := $(SRC_DIR)/aipu_buffer.o \
            $(SRC_DIR)/aipu_cma.o \
            $(SRC_DIR)/aipu_mm.o
HW_OBJ   := $(SRC_DIR)/aipu_core.o \
            $(SRC_DIR)/aipu_ctrl.o \
            $(SRC_DIR)/aipu_io.o \
            $(SRC_DIR)/aipu_irq.o
DRV_OBJ  := $(SRC_DIR)/aipu_common.o
JOB_OBJ  := $(SRC_DIR)/aipu_job_manager.o \
            $(SRC_DIR)/aipu_job_desc.o
MISC_OBJ := $(SRC_DIR)/aipu_error.o \
            $(SRC_DIR)/aipu_profiling.o
VERSION_OBJ := $(SRC_DIR)/zhouyiv1.o

INIT_OBJ := $(SRC_DIR)/platform_driver_init.o
OBJS     := $(FOPS_OBJ) \
            $(MEM_OBJ) \
            $(HW_OBJ) \
            $(DRV_OBJ) \
            $(JOB_OBJ) \
            $(MISC_OBJ) \
            $(VERSION_OBJ)


obj-$(CONFIG_AIPU) += aipu.o
aipu-objs += $(INIT_OBJ)
aipu-objs += $(OBJS)
