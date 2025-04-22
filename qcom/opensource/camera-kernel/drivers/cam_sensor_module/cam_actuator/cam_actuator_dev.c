// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2017-2021, The Linux Foundation. All rights reserved.
 */

#include "cam_actuator_dev.h"
#include "cam_req_mgr_dev.h"
#include "cam_actuator_soc.h"
#include "cam_actuator_core.h"
#include "cam_trace.h"
#include "camera_main.h"

static struct class *actuator_uw_class;
static struct cam_actuator_ctrl_t       *uw_a_ctrl = NULL;
static int is_power_on = 0;
static int is_vcm_init = 0;

static int cam_actuator_subdev_close_internal(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	struct cam_actuator_ctrl_t *a_ctrl =
		v4l2_get_subdevdata(sd);

	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "a_ctrl ptr is NULL");
		return -EINVAL;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	cam_actuator_shutdown(a_ctrl);
	mutex_unlock(&(a_ctrl->actuator_mutex));

	return 0;
}

static int cam_actuator_subdev_close(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh)
{
	bool crm_active = cam_req_mgr_is_open();

	if (crm_active) {
		CAM_DBG(CAM_ACTUATOR,
			"CRM is ACTIVE, close should be from CRM");
		return 0;
	}

	return cam_actuator_subdev_close_internal(sd, fh);
}

static long cam_actuator_subdev_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, void *arg)
{
	int rc = 0;
	struct cam_actuator_ctrl_t *a_ctrl =
		v4l2_get_subdevdata(sd);

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		rc = cam_actuator_driver_cmd(a_ctrl, arg);
		if (rc)
			CAM_ERR(CAM_ACTUATOR,
				"Failed for driver_cmd: %d", rc);
		break;
	case CAM_SD_SHUTDOWN:
		if (!cam_req_mgr_is_shutdown()) {
			CAM_ERR(CAM_CORE, "SD shouldn't come from user space");
			return 0;
		}

		rc = cam_actuator_subdev_close_internal(sd, NULL);
		break;
	default:
		CAM_ERR(CAM_ACTUATOR, "Invalid ioctl cmd: %u", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}
	return rc;
}

#ifdef CONFIG_COMPAT
static long cam_actuator_init_subdev_do_ioctl(struct v4l2_subdev *sd,
	unsigned int cmd, unsigned long arg)
{
	struct cam_control cmd_data;
	int32_t rc = 0;

	if (copy_from_user(&cmd_data, (void __user *)arg,
		sizeof(cmd_data))) {
		CAM_ERR(CAM_ACTUATOR,
			"Failed to copy from user_ptr=%pK size=%zu",
			(void __user *)arg, sizeof(cmd_data));
		return -EFAULT;
	}

	switch (cmd) {
	case VIDIOC_CAM_CONTROL:
		cmd = VIDIOC_CAM_CONTROL;
		rc = cam_actuator_subdev_ioctl(sd, cmd, &cmd_data);
		if (rc) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed in actuator subdev handling rc: %d",
				rc);
			return rc;
		}
		break;
	default:
		CAM_ERR(CAM_ACTUATOR, "Invalid compat ioctl: %d", cmd);
		rc = -ENOIOCTLCMD;
		break;
	}

	if (!rc) {
		if (copy_to_user((void __user *)arg, &cmd_data,
			sizeof(cmd_data))) {
			CAM_ERR(CAM_ACTUATOR,
				"Failed to copy to user_ptr=%pK size=%zu",
				(void __user *)arg, sizeof(cmd_data));
			rc = -EFAULT;
		}
	}
	return rc;
}
#endif

static struct v4l2_subdev_core_ops cam_actuator_subdev_core_ops = {
	.ioctl = cam_actuator_subdev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = cam_actuator_init_subdev_do_ioctl,
#endif
};

static struct v4l2_subdev_ops cam_actuator_subdev_ops = {
	.core = &cam_actuator_subdev_core_ops,
};

static const struct v4l2_subdev_internal_ops cam_actuator_internal_ops = {
	.close = cam_actuator_subdev_close,
};

static int cam_actuator_init_subdev(struct cam_actuator_ctrl_t *a_ctrl)
{
	int rc = 0;

	a_ctrl->v4l2_dev_str.internal_ops =
		&cam_actuator_internal_ops;
	a_ctrl->v4l2_dev_str.ops =
		&cam_actuator_subdev_ops;
	strlcpy(a_ctrl->device_name, CAMX_ACTUATOR_DEV_NAME,
		sizeof(a_ctrl->device_name));
	a_ctrl->v4l2_dev_str.name =
		a_ctrl->device_name;
	a_ctrl->v4l2_dev_str.sd_flags =
		(V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS);
	a_ctrl->v4l2_dev_str.ent_function =
		CAM_ACTUATOR_DEVICE_TYPE;
	a_ctrl->v4l2_dev_str.token = a_ctrl;
	a_ctrl->v4l2_dev_str.close_seq_prior =
		 CAM_SD_CLOSE_MEDIUM_PRIORITY;

	rc = cam_register_subdev(&(a_ctrl->v4l2_dev_str));
	if (rc)
		CAM_ERR(CAM_ACTUATOR,
			"Fail with cam_register_subdev rc: %d", rc);

	return rc;
}

static int cam_actuator_i2c_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int32_t                          rc = 0;
	int32_t                          i = 0;
	struct i2c_client               *client;
	struct cam_actuator_ctrl_t      *a_ctrl;
	struct cam_hw_soc_info          *soc_info = NULL;
	struct cam_actuator_soc_private *soc_private = NULL;

	client = container_of(dev, struct i2c_client, dev);
	if (!client) {
		CAM_ERR(CAM_ACTUATOR,
			"Failed to get i2c client");
		return -EFAULT;
	}

	/* Create sensor control structure */
	a_ctrl = kzalloc(sizeof(*a_ctrl), GFP_KERNEL);
	if (!a_ctrl)
		return -ENOMEM;

	i2c_set_clientdata(client, a_ctrl);

	soc_private = kzalloc(sizeof(struct cam_actuator_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_ctrl;
	}
	a_ctrl->soc_info.soc_private = soc_private;

	a_ctrl->io_master_info.client = client;
	soc_info = &a_ctrl->soc_info;
	soc_info->dev = &client->dev;
	soc_info->dev_name = client->name;
	a_ctrl->io_master_info.master_type = I2C_MASTER;

	rc = cam_actuator_parse_dt(a_ctrl, &client->dev);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "failed: cam_sensor_parse_dt rc %d", rc);
		goto free_soc;
	}

	rc = cam_actuator_init_subdev(a_ctrl);
	if (rc)
		goto free_soc;

	if (soc_private->i2c_info.slave_addr != 0)
		a_ctrl->io_master_info.client->addr =
			soc_private->i2c_info.slave_addr;

	a_ctrl->i2c_data.per_frame =
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (a_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto unreg_subdev;
	}

	INIT_LIST_HEAD(&(a_ctrl->i2c_data.init_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(a_ctrl->i2c_data.per_frame[i].list_head));

	a_ctrl->bridge_intf.device_hdl = -1;
	a_ctrl->bridge_intf.link_hdl = -1;
	a_ctrl->bridge_intf.ops.get_dev_info =
		cam_actuator_publish_dev_info;
	a_ctrl->bridge_intf.ops.link_setup =
		cam_actuator_establish_link;
	a_ctrl->bridge_intf.ops.apply_req =
		cam_actuator_apply_request;
	a_ctrl->last_flush_req = 0;
	a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;

	return rc;

unreg_subdev:
	cam_unregister_subdev(&(a_ctrl->v4l2_dev_str));
free_soc:
	kfree(soc_private);
free_ctrl:
	kfree(a_ctrl);
	return rc;
}

static void cam_actuator_i2c_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct i2c_client               *client = NULL;
	struct cam_actuator_ctrl_t      *a_ctrl = NULL;
	struct cam_actuator_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;

	client = container_of(dev, struct i2c_client, dev);
	if (!client) {
		CAM_ERR(CAM_ACTUATOR,
			"Failed to get i2c client");
		return;
	}

	a_ctrl = i2c_get_clientdata(client);
	/* Handle I2C Devices */
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Actuator device is NULL");
		return;
	}

	CAM_INFO(CAM_ACTUATOR, "i2c remove invoked");
	mutex_lock(&(a_ctrl->actuator_mutex));
	cam_actuator_shutdown(a_ctrl);
	mutex_unlock(&(a_ctrl->actuator_mutex));
	cam_unregister_subdev(&(a_ctrl->v4l2_dev_str));
	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	/*Free Allocated Mem */
	kfree(a_ctrl->i2c_data.per_frame);
	a_ctrl->i2c_data.per_frame = NULL;
	a_ctrl->soc_info.soc_private = NULL;
	v4l2_set_subdevdata(&a_ctrl->v4l2_dev_str.sd, NULL);
	kfree(a_ctrl);
}

const static struct component_ops cam_actuator_i2c_component_ops = {
	.bind = cam_actuator_i2c_component_bind,
	.unbind = cam_actuator_i2c_component_unbind,
};

static int32_t cam_actuator_driver_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	int rc = 0;

	if (client == NULL || id == NULL) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args client: %pK id: %pK",
			client, id);
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		CAM_ERR(CAM_ACTUATOR, "%s :: i2c_check_functionality failed",
			 client->name);
		return -EFAULT;
	}

	CAM_DBG(CAM_ACTUATOR, "Adding sensor actuator component");
	rc = component_add(&client->dev, &cam_actuator_i2c_component_ops);
	if (rc)
		CAM_ERR(CAM_ACTUATOR, "failed to add component rc: %d", rc);

	return rc;
}

static int32_t cam_actuator_driver_i2c_remove(
	struct i2c_client *client)
{
	component_del(&client->dev, &cam_actuator_i2c_component_ops);
	return 0;
}

/*******************************************************************************
 * node
 * Path: /sys/class/actuator_uw/setCode
 * ****************************************************************************/
static int cam_actuator_write_i2c_data8(struct camera_io_master *io_master_info,
	uint32_t addr, uint32_t addr_type, uint8_t *data, uint32_t num_byte)
{
	int32_t ret = 0;
	int32_t cnt;
	struct cam_sensor_i2c_reg_setting i2c_reg_setting;

	if (io_master_info == NULL || data == NULL) {
		CAM_ERR(CAM_ACTUATOR, "Invalid Args o_ctrl, data");
		return -EINVAL;
	}

	i2c_reg_setting.addr_type = addr_type;
	i2c_reg_setting.data_type = CAMERA_SENSOR_I2C_TYPE_BYTE;
	i2c_reg_setting.size = num_byte;
	i2c_reg_setting.delay = 0;
	i2c_reg_setting.reg_setting = (struct cam_sensor_i2c_reg_array *)
			kzalloc(sizeof(struct cam_sensor_i2c_reg_array) *
							num_byte, GFP_KERNEL);
	if (i2c_reg_setting.reg_setting == NULL) {
		CAM_ERR(CAM_ACTUATOR, "kzalloc failed");
		return -ENOMEM;
	}
	i2c_reg_setting.reg_setting[0].reg_addr = addr;
	i2c_reg_setting.reg_setting[0].reg_data = data[0];
	i2c_reg_setting.reg_setting[0].delay = 0;
	i2c_reg_setting.reg_setting[0].data_mask = 0;
	for (cnt = 1; cnt < num_byte; cnt++) {
		i2c_reg_setting.reg_setting[cnt].reg_addr = 0;
		i2c_reg_setting.reg_setting[cnt].reg_data = data[cnt];
		i2c_reg_setting.reg_setting[cnt].delay = 0;
		i2c_reg_setting.reg_setting[cnt].data_mask = 0;
	}
	ret = camera_io_dev_write(io_master_info,
					   &i2c_reg_setting);
	if (ret < 0)
		CAM_ERR(CAM_ACTUATOR, "err! ret:%d", ret);
	kfree(i2c_reg_setting.reg_setting);

	return ret;
}

static void init_VCM()
{
	uint8_t init_addr[] = {0xED, 0x02, 0x02, 0x06, 0x07, 0x08};
	uint8_t init_data[] = {0xAB, 0x01, 0x00, 0x84, 0x01, 0x24};
	int32_t n = 0;

	CAM_INFO(CAM_ACTUATOR, "init_VCM");

	for (n = 0; n < 6; n++) {
		int32_t ret = cam_actuator_write_i2c_data8(&uw_a_ctrl->io_master_info, init_addr[n], CAMERA_SENSOR_I2C_TYPE_BYTE, &init_data[n],1);
		if (ret < 0) {
			CAM_ERR(CAM_ACTUATOR, "setCode ERROR: camera init write ret =%d",ret);
		}
	}
	is_vcm_init = 1;
}

static void setCode(uint32_t reg_val, int32_t step, bool is_exit)
{
	struct cam_sensor_i2c_reg_setting  i2c_reg_settings = {0};
	struct cam_sensor_i2c_reg_array    i2c_reg_array = {0};
	int32_t ret = 0;
	int32_t i = 0;
	int32_t stepcode = reg_val/step;
	int32_t step_val = 0;

	if (is_vcm_init == 0)
	{
		init_VCM();
	}

	CAM_INFO(CAM_ACTUATOR, "actuator_setCode:%d step:%d",reg_val, step);

	for (i=1; i <= step; i++)
	{
		if (!is_exit)
		{
			step_val = i*stepcode;
		}
		else
		{
			step_val = reg_val - i*stepcode;
		}

		i2c_reg_array.reg_addr =0x03;
		i2c_reg_array.reg_data = step_val;
		i2c_reg_array.delay = 0;
		i2c_reg_settings.addr_type =CAMERA_SENSOR_I2C_TYPE_BYTE;
		i2c_reg_settings.data_type =CAMERA_SENSOR_I2C_TYPE_WORD;
		i2c_reg_settings.size = 1;
		i2c_reg_settings.reg_setting = &i2c_reg_array;

		ret = camera_io_dev_write(&uw_a_ctrl->io_master_info,&i2c_reg_settings);
		if (ret < 0) {
			CAM_ERR(CAM_ACTUATOR, "actuator_move_store: camera_io_dev_write ret =%d",ret);
		}
		CAM_DBG(CAM_ACTUATOR, "actuator_setCode:%d", step_val);
		msleep(5);
	}
}

static ssize_t setCode_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	/*int32_t ret = 0;
	uint32_t reg_val = 0;

	ret = kstrtou32(buf, 0, &reg_val);
	if (ret < 0)
	{
		return ret;
	}

	if (reg_val < 0 || reg_val > 1023) {
		CAM_ERR(CAM_ACTUATOR, "cam actuator Wrong input value,must 0 < value < 1024");
		return -ENOMEM;
	}

	if (!uw_a_ctrl->io_master_info.cci_client) {
		ret = -ENOMEM;
		return ret;
	}

	if (is_power_on != 1) {
		CAM_ERR(CAM_ACTUATOR, "cam actuator no power on");
		return -ENOMEM;
	}

	CAM_DBG(CAM_ACTUATOR, "setCode: salve_add[0x%x] i2c_freq_mode[%d] data[%0x]",
		uw_a_ctrl->io_master_info.cci_client->sid,
		uw_a_ctrl->io_master_info.cci_client->i2c_freq_mode,
		reg_val);

	if (is_vcm_init == 0)
	{
		init_VCM();
	}

	setCode(reg_val, 2, false);*/

	return count;
}

static ssize_t powerOn_store(struct class *class, struct class_attribute *attr,
						const char *buf, size_t count)
{
	uint32_t power_status = 0;
	int ret = 0;

	ret = kstrtou32(buf, 0, &power_status);
	if (ret < 0) {
		CAM_ERR(CAM_ACTUATOR, "cam_actuator_power_on Number of parameters error");
		return ret;
	}

	if (uw_a_ctrl != NULL) {
		if (uw_a_ctrl->io_master_info.cci_client->sid == 0x0) {
			uw_a_ctrl->io_master_info.cci_client->sid = 0x18 >> 1;
			uw_a_ctrl->io_master_info.cci_client->i2c_freq_mode = 1;
		}

		CAM_DBG(CAM_ACTUATOR, "uw_a_ctrl: salve_add[0x%x] i2c_freq_mode[%d] NAME[%s]",
			uw_a_ctrl->io_master_info.cci_client->sid,
			uw_a_ctrl->io_master_info.cci_client->i2c_freq_mode,
			uw_a_ctrl->soc_info.dev_name);
	} else {
		CAM_ERR(CAM_ACTUATOR, "uw_a_ctrl is NULL");
		return -ENOMEM;
	}

	if (power_status == 1)
	{
		ret = cam_actuator_power_on_from_other(uw_a_ctrl);
		if (ret < 0 ) {
			CAM_ERR(CAM_ACTUATOR, "cam_actuator_power_on failed ret =%d",ret);
			return ret;
		}
		is_power_on = 1;
		setCode(400, 2, false);
		CAM_DBG(CAM_ACTUATOR, "cam_actuator_power_on success");
	}
	else if (power_status == 0)
	{
		if(is_power_on > 0)
		{
			setCode(400, 40, true);
			ret = cam_actuator_power_off_from_other(uw_a_ctrl);
			if (ret < 0 ) {
				CAM_ERR(CAM_ACTUATOR, "cam_actuator_power_off failed ret =%d",ret);
			}
			is_power_on = 0;
			is_vcm_init = 0;
			CAM_DBG(CAM_ACTUATOR, "cam_actuator_power_off");
		}
	}
	else
	{
		CAM_ERR(CAM_ACTUATOR, "poweron status error: %d", power_status);
		return -EPERM;
	}

	return count;
}

static CLASS_ATTR_ACTUATOR(setCode);
static CLASS_ATTR_ACTUATOR(powerOn);

static int actuator_uw_create_sysfs(void)
{
	int ret = 0;

	if (!actuator_uw_class) {
		actuator_uw_class = class_create(THIS_MODULE, "actuator_uw");

		ret = class_create_file(actuator_uw_class, &class_attr_setCode);
		if (ret < 0) {
			CAM_ERR(CAM_ACTUATOR, "Create reg failed, ret: %d", ret);
			return ret;
		}
		ret = class_create_file(actuator_uw_class, &class_attr_powerOn);
		if (ret < 0) {
			CAM_ERR(CAM_ACTUATOR, "Create reg failed, ret: %d", ret);
			return ret;
		}
	}

	CAM_INFO(CAM_ACTUATOR, "Creat sysfs dev success.");

	return 0;
}

static void actuator_uw_destroy_sysfs(void)
{
	if (actuator_uw_class) {
		class_remove_file(actuator_uw_class, &class_attr_setCode);
		class_remove_file(actuator_uw_class, &class_attr_powerOn);

		class_destroy(actuator_uw_class);
		actuator_uw_class = NULL;
		is_power_on = 0;
		is_vcm_init = 0;
		CAM_DBG(CAM_ACTUATOR, "destroy actuator_uw_class done!");
	}
}

///////////////////////////////////////////device node end

static int cam_actuator_platform_component_bind(struct device *dev,
	struct device *master_dev, void *data)
{
	int32_t                          rc = 0;
	int32_t                          i = 0;
	struct cam_actuator_ctrl_t       *a_ctrl = NULL;
	struct cam_actuator_soc_private  *soc_private = NULL;
	struct platform_device *pdev = to_platform_device(dev);
	char *wu_actuator = "ac15000.qcom,cci0:qcom,actuator2";

	/* Create actuator control structure */
	a_ctrl = devm_kzalloc(&pdev->dev,
		sizeof(struct cam_actuator_ctrl_t), GFP_KERNEL);
	if (!a_ctrl)
		return -ENOMEM;

	/*fill in platform device*/
	a_ctrl->v4l2_dev_str.pdev = pdev;
	a_ctrl->soc_info.pdev = pdev;
	a_ctrl->soc_info.dev = &pdev->dev;
	a_ctrl->soc_info.dev_name = pdev->name;
	a_ctrl->io_master_info.master_type = CCI_MASTER;

	a_ctrl->io_master_info.cci_client = kzalloc(sizeof(
		struct cam_sensor_cci_client), GFP_KERNEL);
	if (!(a_ctrl->io_master_info.cci_client)) {
		rc = -ENOMEM;
		goto free_ctrl;
	}

	soc_private = kzalloc(sizeof(struct cam_actuator_soc_private),
		GFP_KERNEL);
	if (!soc_private) {
		rc = -ENOMEM;
		goto free_cci_client;
	}
	a_ctrl->soc_info.soc_private = soc_private;
	soc_private->power_info.dev = &pdev->dev;

	a_ctrl->i2c_data.per_frame =
		kzalloc(sizeof(struct i2c_settings_array) *
		MAX_PER_FRAME_ARRAY, GFP_KERNEL);
	if (a_ctrl->i2c_data.per_frame == NULL) {
		rc = -ENOMEM;
		goto free_soc;
	}

	INIT_LIST_HEAD(&(a_ctrl->i2c_data.init_settings.list_head));

	for (i = 0; i < MAX_PER_FRAME_ARRAY; i++)
		INIT_LIST_HEAD(&(a_ctrl->i2c_data.per_frame[i].list_head));

	rc = cam_actuator_parse_dt(a_ctrl, &(pdev->dev));
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR, "Paring actuator dt failed rc %d", rc);
		goto free_mem;
	}

	/* Fill platform device id*/
	pdev->id = a_ctrl->soc_info.index;

	rc = cam_actuator_init_subdev(a_ctrl);
	if (rc)
		goto free_mem;

	a_ctrl->bridge_intf.device_hdl = -1;
	a_ctrl->bridge_intf.link_hdl = -1;
	a_ctrl->bridge_intf.ops.get_dev_info =
		cam_actuator_publish_dev_info;
	a_ctrl->bridge_intf.ops.link_setup =
		cam_actuator_establish_link;
	a_ctrl->bridge_intf.ops.apply_req =
		cam_actuator_apply_request;
	a_ctrl->bridge_intf.ops.flush_req =
		cam_actuator_flush_request;
	a_ctrl->last_flush_req = 0;

	platform_set_drvdata(pdev, a_ctrl);
	a_ctrl->cam_act_state = CAM_ACTUATOR_INIT;
	CAM_DBG(CAM_ACTUATOR, "Component bound successfully %d",
		a_ctrl->soc_info.index);

	if (strcmp(wu_actuator, a_ctrl->soc_info.dev_name) == 0) {
		uw_a_ctrl = a_ctrl;
		actuator_uw_create_sysfs();
	}

	return rc;

free_mem:
	kfree(a_ctrl->i2c_data.per_frame);
free_soc:
	kfree(soc_private);
free_cci_client:
	kfree(a_ctrl->io_master_info.cci_client);
free_ctrl:
	devm_kfree(&pdev->dev, a_ctrl);
	return rc;
}

static void cam_actuator_platform_component_unbind(struct device *dev,
	struct device *master_dev, void *data)
{
	struct cam_actuator_ctrl_t      *a_ctrl;
	struct cam_actuator_soc_private *soc_private;
	struct cam_sensor_power_ctrl_t  *power_info;
	struct platform_device *pdev = to_platform_device(dev);

	a_ctrl = platform_get_drvdata(pdev);
	if (!a_ctrl) {
		CAM_ERR(CAM_ACTUATOR, "Actuator device is NULL");
		return;
	}

	mutex_lock(&(a_ctrl->actuator_mutex));
	cam_actuator_shutdown(a_ctrl);
	mutex_unlock(&(a_ctrl->actuator_mutex));
	cam_unregister_subdev(&(a_ctrl->v4l2_dev_str));

	soc_private =
		(struct cam_actuator_soc_private *)a_ctrl->soc_info.soc_private;
	power_info = &soc_private->power_info;

	CAM_DBG(CAM_ACTUATOR, "a_ctrl->io_master_info.cci_client->sid %d",
		a_ctrl->io_master_info.cci_client->sid);

	if (uw_a_ctrl != NULL) {
		uw_a_ctrl = NULL;
		actuator_uw_destroy_sysfs();
	}

	kfree(a_ctrl->io_master_info.cci_client);
	a_ctrl->io_master_info.cci_client = NULL;
	kfree(a_ctrl->soc_info.soc_private);
	a_ctrl->soc_info.soc_private = NULL;
	kfree(a_ctrl->i2c_data.per_frame);
	a_ctrl->i2c_data.per_frame = NULL;
	v4l2_set_subdevdata(&a_ctrl->v4l2_dev_str.sd, NULL);
	platform_set_drvdata(pdev, NULL);
	devm_kfree(&pdev->dev, a_ctrl);
	CAM_DBG(CAM_ACTUATOR, "Actuator component unbinded");
}

const static struct component_ops cam_actuator_platform_component_ops = {
	.bind = cam_actuator_platform_component_bind,
	.unbind = cam_actuator_platform_component_unbind,
};

static int32_t cam_actuator_platform_remove(
	struct platform_device *pdev)
{
	component_del(&pdev->dev, &cam_actuator_platform_component_ops);
	return 0;
}

static const struct of_device_id cam_actuator_driver_dt_match[] = {
	{.compatible = "qcom,actuator"},
	{}
};

static int32_t cam_actuator_driver_platform_probe(
	struct platform_device *pdev)
{
	int rc = 0;

	CAM_DBG(CAM_ACTUATOR, "Adding sensor actuator component");
	rc = component_add(&pdev->dev, &cam_actuator_platform_component_ops);
	if (rc)
		CAM_ERR(CAM_ACTUATOR, "failed to add component rc: %d", rc);

	return rc;
}

MODULE_DEVICE_TABLE(of, cam_actuator_driver_dt_match);

struct platform_driver cam_actuator_platform_driver = {
	.probe = cam_actuator_driver_platform_probe,
	.driver = {
		.name = "qcom,actuator",
		.owner = THIS_MODULE,
		.of_match_table = cam_actuator_driver_dt_match,
		.suppress_bind_attrs = true,
	},
	.remove = cam_actuator_platform_remove,
};

static const struct i2c_device_id i2c_id[] = {
	{ACTUATOR_DRIVER_I2C, (kernel_ulong_t)NULL},
	{ }
};

static const struct of_device_id cam_actuator_i2c_driver_dt_match[] = {
	{.compatible = "qcom,cam-i2c-actuator"},
	{}
};
MODULE_DEVICE_TABLE(of, cam_actuator_i2c_driver_dt_match);

struct i2c_driver cam_actuator_i2c_driver = {
	.id_table = i2c_id,
	.probe  = cam_actuator_driver_i2c_probe,
	.remove = cam_actuator_driver_i2c_remove,
	.driver = {
		.of_match_table = cam_actuator_i2c_driver_dt_match,
		.owner = THIS_MODULE,
		.name = ACTUATOR_DRIVER_I2C,
		.suppress_bind_attrs = true,
	},
};

int cam_actuator_driver_init(void)
{
	int32_t rc = 0;

	rc = platform_driver_register(&cam_actuator_platform_driver);
	if (rc < 0) {
		CAM_ERR(CAM_ACTUATOR,
			"platform_driver_register failed rc = %d", rc);
		return rc;
	}
	rc = i2c_add_driver(&cam_actuator_i2c_driver);
	if (rc)
		CAM_ERR(CAM_ACTUATOR, "i2c_add_driver failed rc = %d", rc);

	return rc;
}

void cam_actuator_driver_exit(void)
{
	platform_driver_unregister(&cam_actuator_platform_driver);
	i2c_del_driver(&cam_actuator_i2c_driver);
}

MODULE_DESCRIPTION("cam_actuator_driver");
MODULE_LICENSE("GPL v2");
