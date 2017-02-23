#include <linux/delay.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/spinlock.h>

struct mdm_communication_mcu_platform_data{
    unsigned int mdmRstMcu;
	unsigned int mdmWakeupMcu;
	unsigned int mcuWakeupMdm;
	unsigned int mcuBoot;

    unsigned int voteForMdmSleep;
	int mcuWakeupMdmIrq;
	spinlock_t lock;
	
    struct pinctrl *pPinctrl;
	struct input_dev *pwr;
	struct delayed_work	pwr_work;
};

#define POWER2_KEY_STATUS_DELAY	msecs_to_jiffies(250)
#define POWER2_KEY_RELEASE (0)
#define POWER2_KEY_PRESS   (1)

void mdm_communication_mcu_report_power_key(void *dev);

struct input_dev *g_pwr = NULL;


//struct CommunicationMcuData *g_pCommunicationMcuData = NULL;


//int reset_gpio = -1;
//struct kobject *gpio_rstMcu_kobj = NULL;
//struct pinctrl *rstMcu_pinctrl = NULL;

/*static ssize_t sys_reset_gpio_enable( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t count )
{
    pr_info("%s: reset gpio status before reset %d\n", __func__, gpio_get_value(reset_gpio));
    gpio_set_value(reset_gpio, 0);
    //gpio_direction_output(reset_gpio, 0);
    pr_info("%s: reset gpio status on-going reset %d\n", __func__, gpio_get_value(reset_gpio));
    msleep(100); //Wait for GPIO to settle
    gpio_set_value(reset_gpio, 1);
    //gpio_direction_output(reset_gpio, 1);

    pr_info("%s: reset gpio status after reset %d\n", __func__, gpio_get_value(reset_gpio));
    

    return count;
}*/

static ssize_t mdm_communication_mcu_gpio_show( struct device *dev,
                                       struct device_attribute *attr, const char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
	struct mdm_communication_mcu_platform_data *pdata = 
		platform_get_drvdata(pdev);
	
	int pin = 0, value = -1;
	
	if(0 == strcmp("mdmRstMcu", attr->attr.name))
	{
	   pin = pdata->mdmRstMcu;
	}
	else if(0 == strcmp("mdmWakeupMcu", attr->attr.name))
	{
	   pin = pdata->mdmWakeupMcu;
	}
	else if(0 == strcmp("mcuWakeupMdm", attr->attr.name))
	{
	   pin = pdata->mcuWakeupMdm;
	}
	else if(0 == strcmp("mcuBoot", attr->attr.name))
	{
	   pin = pdata->mcuBoot;
	}

	value = gpio_get_value(pin);

    printk(KERN_ERR "%s, name=%s, pin=%d, value=%d.\n",
		__func__, 
		attr->attr.name,
		pin, 
		value); 
    

    return sprintf(buf, "%s:%d\n", attr->attr.name, value);;
}

static ssize_t mdm_communication_mcu_gpio_store( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mdm_communication_mcu_platform_data *pdata = 
		platform_get_drvdata(pdev);

	
    int pin=0, ret=0;
	/*char *after;
	unsigned long value = simple_strtoul(buf, &after, 10);

	size_t count = after - buf;  
    
    if (count == size)  
        ret = count; */ 
    unsigned long value = simple_strtoul(buf, NULL, 10);
	
	if(0 == strcmp("mdmRstMcu", attr->attr.name))
	{
	   pin = pdata->mdmRstMcu;
	}
	else if(0 == strcmp("mdmWakeupMcu", attr->attr.name))
	{
	   pin = pdata->mdmWakeupMcu;
	}
	else if(0 == strcmp("mcuWakeupMdm", attr->attr.name))
	{
	   pin = pdata->mcuWakeupMdm;
	}
	else if(0 == strcmp("mcuBoot", attr->attr.name))
	{
	   pin = pdata->mcuBoot;
	}

	ret = gpio_direction_output(pin, value);

	/*just for test begin*/
    /*if (pin == 13)
    {
        printk(KERN_ERR "%s, Report Value.\n", __func__);
        input_report_key(pdata->pwr, KEY_POWER2, POWER2_KEY_PRESS);
	    input_sync(pdata->pwr);

	    mdelay(200);

	    input_report_key(pdata->pwr, KEY_POWER2, POWER2_KEY_RELEASE);
	    input_sync(pdata->pwr);
    }*/
	/*just for test*/

    printk(KERN_ERR "%s, name=%s, pin=%d, value=%u, size=%d.\n",
		__func__, 
		attr->attr.name,
		pin, 
		value,
		size); 

    return size;

}

static ssize_t mdm_communication_mcu_vote_show( struct device *dev,
                                       struct device_attribute *attr, const char *buf)
{
    struct platform_device *pdev = to_platform_device(dev);
	struct mdm_communication_mcu_platform_data *pdata = 
		platform_get_drvdata(pdev);
	
/*
    printk(KERN_ERR "%s, %s = %u.\n",
		__func__, 
		attr->attr.name,
		pdata->voteForMdmSleep); 
 */   

    return sprintf(buf, "%u\n", pdata->voteForMdmSleep);;
}

static ssize_t mdm_communication_mcu_vote_store( struct device *dev,
                                       struct device_attribute *attr, const char *buf, size_t size )
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mdm_communication_mcu_platform_data *pdata = 
		platform_get_drvdata(pdev);
	unsigned long flags;

    /*printk(KERN_ERR "%s, %s=%u, buf=%d.\n",
		__func__, 
		attr->attr.name,
        pdata->voteForMdmSleep,
        simple_strtoul(buf, NULL, 10));*/
	
	spin_lock_irqsave(&pdata->lock,flags);
    //pdata->voteForMdmSleep = pdata->voteForMdmSleep | (int)simple_strtoul(buf, NULL, 10);
    pdata->voteForMdmSleep = (int)simple_strtoul(buf, NULL, 10);
	spin_unlock_irqrestore(&pdata->lock,flags);
	
/*
	printk(KERN_ERR "%s, voteForMdmSleep=%u, size=%d, sizeof(unsigned int)=%d.\n", 
		__func__, pdata->voteForMdmSleep, size, sizeof(unsigned int));
*/

    return size;
}


static DEVICE_ATTR(mdmRstMcu, S_IWUSR|S_IRUGO, mdm_communication_mcu_gpio_show, mdm_communication_mcu_gpio_store);
static DEVICE_ATTR(mdmWakeupMcu, S_IWUSR|S_IRUGO, mdm_communication_mcu_gpio_show, mdm_communication_mcu_gpio_store);
static DEVICE_ATTR(mcuWakeupMdm, S_IWUSR|S_IRUGO, mdm_communication_mcu_gpio_show, mdm_communication_mcu_gpio_store);
static DEVICE_ATTR(mcuBoot, S_IWUSR|S_IRUGO, mdm_communication_mcu_gpio_show, mdm_communication_mcu_gpio_store);
static DEVICE_ATTR(voteForMdmSleep, S_IWUSR|S_IRUGO, mdm_communication_mcu_vote_show, mdm_communication_mcu_vote_store);



static struct attribute *mdmCommunicationMcu_attributes[] = {
	&dev_attr_mdmRstMcu.attr,
	&dev_attr_mdmWakeupMcu.attr,
	&dev_attr_mcuWakeupMdm.attr,
	&dev_attr_mcuBoot.attr,
	&dev_attr_voteForMdmSleep.attr,
	NULL
};

static struct attribute_group mdmCommunicationMcu_attribute_group = {
	.attrs = mdmCommunicationMcu_attributes
};


static int mdm_communication_mcu_pinctrl_configure(struct pinctrl *pPinctrl, bool active)
{
    struct pinctrl_state *pstate = NULL;
    int retval;

    if (active)
	{
        pstate = pinctrl_lookup_state(pPinctrl, "active");
        pr_info("%s: pinctrl start\n", __func__);
        if (IS_ERR(pstate)) {
            pr_err("%s: cannot get this pinctrl active state\n", __func__);
            return PTR_ERR(pstate);
        }
    } 
	else
	{
		    pstate = pinctrl_lookup_state(pPinctrl, "sleep");
        if (IS_ERR(pstate))
		{
            pr_err("%s: cannot get gpio pinctrl sleep state\n", __func__);
            return PTR_ERR(pstate);
        }
    }
	
    retval = pinctrl_select_state(pPinctrl, pstate);
    printk(KERN_ERR "%s, active=%d, retval=%d.\n", __func__, active, retval);
	
    if (retval)
	{
        pr_err("%s: cannot set ts pinctrl active state\n", __func__);
        return retval;
    }
    return 0;
}


static int mdm_communication_mcu_pinctrl_configure_for_other_gpio(struct pinctrl *pPinctrl, bool active)
{
    struct pinctrl_state *pstate = NULL;
    int retval;

    if (active)
	{
    } 
	else
	{
		pstate = pinctrl_lookup_state(pPinctrl, "ops");//other gpio sleep
        if (IS_ERR(pstate))
		{
            pr_err("%s: cannot get gpio pinctrl sleep state\n", __func__);
            return PTR_ERR(pstate);
        }

		retval = pinctrl_select_state(pPinctrl, pstate);

		printk(KERN_ERR "%s, retval=%d.\n",
			__func__, retval);
		
	    if (retval)
		{
	        pr_err("%s: cannot set other gpio pinctrl state\n", __func__);
	        return retval;
	    }
    }
	
    
	
    pr_info("%s: pinctrl retval=%d.\n", __func__, retval);
    return 0;
}


 
struct mdm_communication_mcu_platform_data
	*mdm_communication_mcu_dt_to_pdata(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct mdm_communication_mcu_platform_data *pdata;
	
	pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		pr_err("unable to allocate memory for platform data\n");
		return ERR_PTR(-ENOMEM);
	}
	
	/* mdm reset mcu GPIO */
	pdata->mdmRstMcu= of_get_named_gpio(node,
					"gpio-mdm-rst-mcu", 0);
	if (pdata->mdmRstMcu < 0)
		pr_err("gpio-mdm-rst-mcu is not available\n");

    /* mdm wakeup mcu GPIO */
	pdata->mdmWakeupMcu= of_get_named_gpio(node,
					"gpio-mdm-wakeup-mcu", 0);
	if (pdata->mdmWakeupMcu < 0)
		pr_err("gpio-mdm-wakeup-mcu is not available\n");

	/* mcu wakeup mdm GPIO */
	pdata->mcuWakeupMdm = of_get_named_gpio(node,
					"gpio-mcu-wakeup-mdm", 0);
	if (pdata->mcuWakeupMdm < 0)
		pr_err("gpio-mcu-wakeup-mdm is not available\n");

	/*mcu boot GPIO*/
	pdata->mcuBoot = of_get_named_gpio(node,
					"gpio-mcu-boot", 0);
	if (pdata->mcuBoot < 0)
		pr_err("gpio-mcu-boot is not available\n");


	printk(KERN_ERR "%s, gpio-mdm-rst-mcu:%d gpio-mdm-wakeup-mcu:%d "
		"gpio-mcu-wakeup-mdm:%d  gpio-mcu-boot:%d \n",
		__func__, pdata->mdmRstMcu, pdata->mdmWakeupMcu,
		pdata->mcuWakeupMdm, pdata->mcuBoot);

	return pdata;
}

static void mdm_communication_mcu_pwr_work_func(struct work_struct *work)
{
	struct mdm_communication_mcu_platform_data *pdata =
		container_of(work, struct mdm_communication_mcu_platform_data, pwr_work.work);
    int value = gpio_get_value(pdata->mcuWakeupMdm);

    printk(KERN_ERR "%s value=%d.\n", __func__, value);
	
    if(!value)
    {
        input_report_key(pdata->pwr, KEY_POWER2, POWER2_KEY_RELEASE);
	    input_sync(pdata->pwr);
		cancel_delayed_work_sync(&pdata->pwr_work);
		 printk(KERN_ERR "%s cancel.", __func__);
    }
	else
	{
	  
	}
}

void mdm_communication_mcu_report_power_key(void *dev)
{
    struct platform_device *pdev = to_platform_device(dev);
	struct mdm_communication_mcu_platform_data *pdata = 
		platform_get_drvdata(pdev);

	input_report_key(g_pwr, KEY_POWER2, POWER2_KEY_PRESS);
	input_sync(g_pwr);
	mdelay(100);
	input_report_key(g_pwr, KEY_POWER2, POWER2_KEY_RELEASE);
	input_sync(g_pwr);
}

static irqreturn_t mdm_communication_mcu_wakeup_irq_isr(int irq, void *dev)
{
	printk(KERN_ERR "Entry  %s, irq=%d.\n", __func__, irq);

	//schedule_delayed_work(&pdata->pwr_work, POWER2_KEY_STATUS_DELAY);
	
	//return IRQ_HANDLED;
	return IRQ_WAKE_THREAD;
}

static irqreturn_t mdm_communication_mcu_wakeup_irq_thread_fn(int irq, void *dev)
{
    printk(KERN_ERR "Entry -------->[ %s ], irq=%d.\n", __func__, irq);
	
	//report event to user space
	mdm_communication_mcu_report_power_key(dev);

	return IRQ_HANDLED;
}


static int mdm_coummunication_mcu_config_gpios(struct platform_device *pdev)
{	
    int ret = -1;
    const struct mdm_communication_mcu_platform_data *pdata =
					pdev->dev.platform_data;

    if (gpio_is_valid(pdata->mdmRstMcu)) {
		ret = gpio_request(pdata->mdmRstMcu,
						"MDM_RST_MCU");
		if (unlikely(ret)) {
			pr_debug("gpio request failed for:%d\n",
				pdata->mdmRstMcu);
			gpio_free(pdata->mdmRstMcu);
		}
		
		gpio_direction_output(pdata->mdmRstMcu, 1);
		gpio_set_value_cansleep(pdata->mdmRstMcu, 1);
	}else {
	    pr_debug("Pdata is NULL.\n");
		ret = -EINVAL;
	}

	if (gpio_is_valid(pdata->mdmWakeupMcu)) {
		ret = gpio_request(pdata->mdmWakeupMcu,
						"MDM_WAKEUP_MCU");
		if (unlikely(ret)) {
			pr_debug("gpio request failed for:%d\n",
				pdata->mdmWakeupMcu);
			gpio_free(pdata->mdmWakeupMcu);
		}
		
		gpio_direction_output(pdata->mdmWakeupMcu, 0);
	}else {
	    pr_debug("Pdata is NULL.\n");
		ret = -EINVAL;
	}

    if (gpio_is_valid(pdata->mcuBoot)) {
		ret = gpio_request(pdata->mcuBoot,
						"MCU_BOOT");
		if (unlikely(ret)) {
			pr_debug("gpio request failed for:%d\n",
				pdata->mcuBoot);
			gpio_free(pdata->mcuBoot);
		}
		
		gpio_direction_output(pdata->mcuBoot, 0);
	}else {
	    pr_debug("Pdata is NULL.\n");
		ret = -EINVAL;
	}

    if (gpio_is_valid(pdata->mcuWakeupMdm)) {
		ret = gpio_request(pdata->mcuWakeupMdm,
						"MCU_WAKEUP_MDM");
		if (unlikely(ret)) {
			pr_debug("gpio request failed for:%d\n",
				pdata->mcuWakeupMdm);
			gpio_free(pdata->mcuWakeupMdm);
		}
		else
		{
		    gpio_direction_input(pdata->mcuWakeupMdm);
            //wakeup interrupt
		    /*request_threaded_irq(gpio_to_irq(pdata->mcuWakeupMdm),
		               mdm_communication_mcu_wakeup_irq_isr,
		               mdm_communication_mcu_wakeup_irq_thread_fn,
		               IRQF_TRIGGER_RISING,
					   //IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					   "mcuWakeupMdmIrq", pdev);*/
			devm_request_irq(&pdev->dev, 
			                 gpio_to_irq(pdata->mcuWakeupMdm),
			                 mdm_communication_mcu_wakeup_irq_thread_fn,
			                 IRQF_TRIGGER_RISING,
			                 "mcuWakeupMdmIrq", pdev);
		}		
	}else {
	    pr_debug("Pdata is NULL.\n");
		ret = -EINVAL;
	}

	printk(KERN_ERR "%s OK.\n", __func__);
	
	return ret;		
}


static int mdm_communication_mcu_probe(struct platform_device *pdev)
{
    int err;
	struct mdm_communication_mcu_platform_data *pdata = pdev->dev.platform_data;

    //parse dt
	if (pdev->dev.of_node) {
		dev_dbg(&pdev->dev, "device tree enabled\n");
		pdata = mdm_communication_mcu_dt_to_pdata(pdev);
		if (IS_ERR(pdata))
			return PTR_ERR(pdata);

		//pdata->voteForMdmSleep = 0xFF00FF00;
		pdata->voteForMdmSleep = 0x00;
		pdev->dev.platform_data = pdata;

		//platform_set_drvdata(pdev, pdata);
	}
   
    //config
    mdm_coummunication_mcu_config_gpios(pdev);
   
    //attribute
	err = sysfs_create_group(&pdev->dev.kobj, &mdmCommunicationMcu_attribute_group);
	if (err) {
		printk(KERN_ERR "%s sysfs create failed: %d\n", __func__, err);
	}

	//register power key for mcu wakeup devices
	pdata->pwr = devm_input_allocate_device(&pdev->dev);
	if (!pdata->pwr) {
		printk(KERN_ERR "%s Can't allocate power button\n", __func__);
	}
	else
	{
	   input_set_capability(pdata->pwr, EV_KEY, KEY_POWER2);

	   pdata->pwr->name = "mcuWakeupMdm";
	   pdata->pwr->phys = "mcuWakeupMdm/input1";

	   err = input_register_device(pdata->pwr);

	   g_pwr = pdata->pwr;

	   INIT_DELAYED_WORK(&pdata->pwr_work, mdm_communication_mcu_pwr_work_func);
	   printk(KERN_ERR "%s register input power2 key, g_pwr=0x%x, pdata->pwr=0x%x.\n",
	   	  __func__, g_pwr, pdata->pwr);
	}	

    //pinctrl config
     pdata->pPinctrl = devm_pinctrl_get(&pdev->dev);
     if (IS_ERR_OR_NULL(pdata->pPinctrl)) {          
        pr_info("%s: Target does not use pinctrl\n", __func__);
     }
	 else
	 {
	     err = mdm_communication_mcu_pinctrl_configure(pdata->pPinctrl, true);

		 err = mdm_communication_mcu_pinctrl_configure_for_other_gpio(pdata->pPinctrl, false);
	 }

	 platform_set_drvdata(pdev, pdata);

     printk(KERN_ERR "%s OK.\n", __func__);
	 return err;
}

static int mdm_communication_mcu_remove(struct platform_device *pdev)
{
	struct mdm_communication_mcu_platform_data *pdata = pdev->dev.platform_data;

    cancel_delayed_work_sync(&pdata->pwr_work);
    input_unregister_device(pdata->pwr);
	sysfs_remove_group(&pdev->dev.kobj, &mdmCommunicationMcu_attribute_group);
	gpio_free(pdata->mcuWakeupMdm);
	gpio_free(pdata->mdmRstMcu);
	gpio_free(pdata->mdmWakeupMcu);
	gpio_free(pdata->mcuBoot);
	g_pwr = NULL;

	printk(KERN_ERR "%s ok.\n", __func__);

    return 0;
}

static int mdm_communication_mcu_suspend(struct device *dev)
{
    printk(KERN_ERR "%s entry.\n", __func__);
	/*struct mdm_communication_mcu_platform_data *pdata = 
	   platform_get_drvdata(to_platform_device(dev));

    mdm_communication_mcu_pinctrl_configure(pdata->pPinctrl, false);*/

    struct platform_device *pdev = to_platform_device(dev);
	struct mdm_communication_mcu_platform_data *pdata = 
		platform_get_drvdata(pdev);
    mdm_communication_mcu_pinctrl_configure(pdata->pPinctrl, false);
    mdm_communication_mcu_pinctrl_configure_for_other_gpio(pdata->pPinctrl, false);
	enable_irq_wake(gpio_to_irq(pdata->mcuWakeupMdm));

    //add for test
	//gpio_direction_input(pdata->mdmRstMcu);
	//gpio_cansleep(pdata->mdmRstMcu);
	//gpio_direction_output(pdata->mdmRstMcu, 0);//ok 3ma
	
	printk(KERN_ERR "%s", __func__);

    return 0;
}

static int mdm_communication_mcu_resume(struct device *dev)
{
    printk(KERN_ERR "%s entry.\n", __func__);
    /*struct mdm_communication_mcu_platform_data *pdata = 
	   platform_get_drvdata(to_platform_device(dev));

	mdm_communication_mcu_pinctrl_configure(pdata->pPinctrl, true);*/

	struct platform_device *pdev = to_platform_device(dev);
	struct mdm_communication_mcu_platform_data *pdata = 
		platform_get_drvdata(pdev);
	disable_irq_wake(gpio_to_irq(pdata->mcuWakeupMdm));
	mdm_communication_mcu_pinctrl_configure(pdata->pPinctrl, true);

	printk(KERN_ERR "%s", __func__);	

    return 0;
}


static struct of_device_id mdmCommunicationMcu_match_table[] = {
{
    .compatible = "qcom,mdm-communication-mcu"},
    {},
};


static const struct dev_pm_ops mdm_communication_mcu_pm_ops = {
         .suspend = mdm_communication_mcu_suspend,
         .resume   = mdm_communication_mcu_resume,
};



static struct platform_driver mdmCommunicationMcu_driver = {
    .probe = mdm_communication_mcu_probe,
    .remove = mdm_communication_mcu_remove,
    .driver = {
        .name = "MdmCommunicationMcu",
        .owner = THIS_MODULE,        
        .pm = &mdm_communication_mcu_pm_ops,
        .of_match_table = mdmCommunicationMcu_match_table,
    },
};

static int __init mdm_communication_mcu_init(void)
{
    return platform_driver_register(&mdmCommunicationMcu_driver);
}

static void __exit mdm_communication_mcu_exit(void)
{
    platform_driver_unregister(&mdmCommunicationMcu_driver);
}

module_init(mdm_communication_mcu_init);
module_exit(mdm_communication_mcu_exit);
MODULE_DESCRIPTION("GPIO RSTMCU");
MODULE_LICENSE("GPL v2");
