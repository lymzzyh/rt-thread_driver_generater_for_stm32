import os
import xml.dom.minidom
import re
import datetime

def get_header_info(Chip):
    return '''
 /*
 * File      : drv_gpio.c
 * 
 * This file is auto generate from cubemx xml for ''' + Chip + ''' 
 *
 * Change Logs:
 * Date           Author            Notes
 * ''' + datetime.datetime.now().strftime('%Y-%m-%d') + '''     ZYH            auto generated
 */
'''

gpio_template_header_string = '''
#include <rthw.h>
#include <rtdevice.h>
#include <board.h>
#ifdef RT_USING_PIN
#define __STM32_PIN(index, gpio, gpio_index) (gpio | gpio_index)
#define __STM32_PIN_DEFAULT 0

#define A   (1U << 8)
#define B   (2U << 8)
#define C   (3U << 8)
#define D   (4U << 8)
#define E   (5U << 8)
#define F   (6U << 8)
#define G   (7U << 8)
#define H   (8U << 8)
#define I   (9U << 8)
#define J   (10U << 8)
#define K   (11U << 8)

static GPIO_TypeDef * get_st_gpio(rt_uint16_t gpio_pin)
{
    switch(gpio_pin & 0xFF00)
    {
    case A:
        #ifdef GPIOA
        return GPIOA;
        #endif
    case B:
        #ifdef GPIOB
        return GPIOB;
        #endif
    case C:
        #ifdef GPIOC
        return GPIOC;
        #endif
    case D:
        #ifdef GPIOD
        return GPIOD;
        #endif
    case E:
        #ifdef GPIOE
        return GPIOE;
        #endif
    case F:
        #ifdef GPIOF
        return GPIOF;
        #endif
    case G:
        #ifdef GPIOG
        return GPIOG;
        #endif
    case H:
        #ifdef GPIOH
        return GPIOH;
        #endif
    case I:
        #ifdef GPIOI
        return GPIOI;
        #endif
    case J:
        #ifdef GPIOJ
        return GPIOJ;
        #endif
    case K:
        #ifdef GPIOK
        return GPIOK;
        #endif
    default:
        return RT_NULL;
    }
}

#define get_st_pin(gpio_pin) (0x01 << (gpio_pin&0xFF))

static void drv_clock_enable(rt_uint16_t gpio_pin)
{
    switch(gpio_pin & 0xFF00)
    {
    case A:
        #ifdef __HAL_RCC_GPIOA_CLK_ENABLE
        __HAL_RCC_GPIOA_CLK_ENABLE();
        #endif
        break;
    case B:
        #ifdef __HAL_RCC_GPIOB_CLK_ENABLE
        __HAL_RCC_GPIOB_CLK_ENABLE();
        #endif
        break;
    case C:
        #ifdef __HAL_RCC_GPIOC_CLK_ENABLE
        __HAL_RCC_GPIOC_CLK_ENABLE();
        #endif
        break;
    case D:
        #ifdef __HAL_RCC_GPIOD_CLK_ENABLE
        __HAL_RCC_GPIOD_CLK_ENABLE();
        #endif
        break;
    case E:
        #ifdef __HAL_RCC_GPIOE_CLK_ENABLE
        __HAL_RCC_GPIOE_CLK_ENABLE();
        #endif
        break;
    case F:
        #ifdef __HAL_RCC_GPIOF_CLK_ENABLE
        __HAL_RCC_GPIOF_CLK_ENABLE();
        #endif
        break;
    case G:
        #ifdef __HAL_RCC_GPIOG_CLK_ENABLE
        __HAL_RCC_GPIOG_CLK_ENABLE();
        #endif
        break;
    case H:
        #ifdef __HAL_RCC_GPIOH_CLK_ENABLE
        __HAL_RCC_GPIOH_CLK_ENABLE();
        #endif
        break;
    case I:
        #ifdef __HAL_RCC_GPIOI_CLK_ENABLE
        __HAL_RCC_GPIOI_CLK_ENABLE();
        #endif
        break;
    case J:
        #ifdef __HAL_RCC_GPIOJ_CLK_ENABLE
        __HAL_RCC_GPIOJ_CLK_ENABLE();
        #endif
        break;
    case K:
        #ifdef __HAL_RCC_GPIOK_CLK_ENABLE
        __HAL_RCC_GPIOK_CLK_ENABLE();
        #endif
        break;
    default:
        break;
    }
}

/* STM32 GPIO driver */
static const rt_uint16_t pins[] =
{
'''

gpio_template_end_string = '''
};

struct pin_irq_map
{
    rt_uint16_t pinbit;
    IRQn_Type irqno;
};

static const struct pin_irq_map pin_irq_map[] =
{
    {GPIO_PIN_0, EXTI0_IRQn},
    {GPIO_PIN_1, EXTI1_IRQn},
    {GPIO_PIN_2, EXTI2_IRQn},
    {GPIO_PIN_3, EXTI3_IRQn},
    {GPIO_PIN_4, EXTI4_IRQn},
    {GPIO_PIN_5, EXTI9_5_IRQn},
    {GPIO_PIN_6, EXTI9_5_IRQn},
    {GPIO_PIN_7, EXTI9_5_IRQn},
    {GPIO_PIN_8, EXTI9_5_IRQn},
    {GPIO_PIN_9, EXTI9_5_IRQn},
    {GPIO_PIN_10, EXTI15_10_IRQn},
    {GPIO_PIN_11, EXTI15_10_IRQn},
    {GPIO_PIN_12, EXTI15_10_IRQn},
    {GPIO_PIN_13, EXTI15_10_IRQn},
    {GPIO_PIN_14, EXTI15_10_IRQn},
    {GPIO_PIN_15, EXTI15_10_IRQn},
};

struct rt_pin_irq_hdr pin_irq_hdr_tab[] =
{
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
    {-1, 0, RT_NULL, RT_NULL},
};

#define ITEM_NUM(items) sizeof(items) / sizeof(items[0])
static rt_uint16_t get_pin(uint8_t pin)
{
    rt_uint16_t gpio_pin = __STM32_PIN_DEFAULT;
    if (pin < ITEM_NUM(pins))
    {
        gpio_pin = pins[pin];
    }
    return gpio_pin;
};

static void stm32_pin_write(rt_device_t dev, rt_base_t pin, rt_base_t value)
{
    rt_uint16_t gpio_pin;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return;
    }
    HAL_GPIO_WritePin(get_st_gpio(gpio_pin), get_st_pin(gpio_pin), (GPIO_PinState)value);
}

static int stm32_pin_read(rt_device_t dev, rt_base_t pin)
{
    rt_uint16_t gpio_pin;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return PIN_LOW;
    }
    return HAL_GPIO_ReadPin(get_st_gpio(gpio_pin), get_st_pin(gpio_pin));
}

static void stm32_pin_mode(rt_device_t dev, rt_base_t pin, rt_base_t mode)
{
    rt_uint16_t gpio_pin;
    GPIO_InitTypeDef GPIO_InitStruct;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return;
    }
    /* GPIO Periph clock enable */
    drv_clock_enable(gpio_pin);
    /* Configure GPIO_InitStructure */
    GPIO_InitStruct.Pin = get_st_pin(gpio_pin);
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    if (mode == PIN_MODE_INPUT)
    {
        /* input setting: not pull. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    }
    else if (mode == PIN_MODE_INPUT_PULLUP)
    {
        /* input setting: pull up. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
    }
    else if (mode == PIN_MODE_INPUT_PULLDOWN)
    {
        /* input setting: pull down. */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    }
    else if (mode == PIN_MODE_OUTPUT_OD)
    {
        /* output setting: od. */
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    }
    HAL_GPIO_Init(get_st_gpio(gpio_pin), &GPIO_InitStruct);
}

static const struct pin_irq_map *get_pin_irq_map(rt_uint16_t gpio_pin)
{
    rt_int32_t mapindex = gpio_pin & 0xFF;
    if (mapindex < 0 || mapindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_NULL;
    }
    return &pin_irq_map[mapindex];
};

static rt_err_t stm32_pin_attach_irq(struct rt_device *device, rt_int32_t pin,
                              rt_uint32_t mode, void (*hdr)(void *args), void *args)
{
    rt_uint16_t gpio_pin;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    GPIO_InitTypeDef GPIO_InitStruct;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return RT_ENOSYS;
    }
    irqindex = gpio_pin&0xFF;
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }
    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == pin &&
            pin_irq_hdr_tab[irqindex].hdr == hdr &&
            pin_irq_hdr_tab[irqindex].mode == mode &&
            pin_irq_hdr_tab[irqindex].args == args)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    if (pin_irq_hdr_tab[irqindex].pin != -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EBUSY;
    }
    pin_irq_hdr_tab[irqindex].pin = pin;
    pin_irq_hdr_tab[irqindex].hdr = hdr;
    pin_irq_hdr_tab[irqindex].mode = mode;
    pin_irq_hdr_tab[irqindex].args = args;
    rt_hw_interrupt_enable(level);
    
    /* GPIO Periph clock enable */
    drv_clock_enable(gpio_pin);
    /* Configure GPIO_InitStructure */
    GPIO_InitStruct.Pin = get_st_pin(gpio_pin);
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    switch (pin_irq_hdr_tab[irqindex].mode)
    {
    case PIN_IRQ_MODE_RISING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
        break;
    case PIN_IRQ_MODE_FALLING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
        break;
    case PIN_IRQ_MODE_RISING_FALLING:
        GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
        break;
    }
    HAL_GPIO_Init(get_st_gpio(gpio_pin), &GPIO_InitStruct);
    
    return RT_EOK;
}

static rt_err_t stm32_pin_detach_irq(struct rt_device *device, rt_int32_t pin)
{
    rt_uint16_t gpio_pin;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return RT_ENOSYS;
    }
    irqindex = gpio_pin&0xFF;
    if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
    {
        return RT_ENOSYS;
    }
    level = rt_hw_interrupt_disable();
    if (pin_irq_hdr_tab[irqindex].pin == -1)
    {
        rt_hw_interrupt_enable(level);
        return RT_EOK;
    }
    pin_irq_hdr_tab[irqindex].pin = -1;
    pin_irq_hdr_tab[irqindex].hdr = RT_NULL;
    pin_irq_hdr_tab[irqindex].mode = 0;
    pin_irq_hdr_tab[irqindex].args = RT_NULL;
    rt_hw_interrupt_enable(level);
    return RT_EOK;
}

static rt_err_t stm32_pin_irq_enable(struct rt_device *device, rt_base_t pin,
                              rt_uint32_t enabled)
{
    rt_uint16_t gpio_pin;
    const struct pin_irq_map *irqmap;
    rt_base_t level;
    rt_int32_t irqindex = -1;
    
    gpio_pin = get_pin(pin);
    if (get_st_gpio(gpio_pin) == RT_NULL)
    {
        return RT_ENOSYS;
    }
    if (enabled == PIN_IRQ_ENABLE)
    {
        irqindex = gpio_pin&0xFF;
        if (irqindex < 0 || irqindex >= ITEM_NUM(pin_irq_map))
        {
            return RT_ENOSYS;
        }
        level = rt_hw_interrupt_disable();
        if (pin_irq_hdr_tab[irqindex].pin == -1)
        {
            rt_hw_interrupt_enable(level);
            return RT_ENOSYS;
        }
        irqmap = &pin_irq_map[irqindex];
        HAL_NVIC_ClearPendingIRQ(irqmap->irqno);
        __HAL_GPIO_EXTI_CLEAR_IT(irqindex);
        __HAL_GPIO_EXTI_CLEAR_FLAG(irqindex);
        HAL_NVIC_SetPriority(irqmap->irqno, 5, 0);
        HAL_NVIC_EnableIRQ(irqmap->irqno);
        rt_hw_interrupt_enable(level);
    }
    else if (enabled == PIN_IRQ_DISABLE)
    {
        irqmap = get_pin_irq_map(gpio_pin);
        if (irqmap == RT_NULL)
        {
            return RT_ENOSYS;
        }
        HAL_NVIC_DisableIRQ(irqmap->irqno);
        HAL_NVIC_ClearPendingIRQ(irqmap->irqno);
    }
    else
    {
        return RT_ENOSYS;
    }
    return RT_EOK;
}

const static struct rt_pin_ops _stm32_pin_ops =
{
    stm32_pin_mode,
    stm32_pin_write,
    stm32_pin_read,
    stm32_pin_attach_irq,
    stm32_pin_detach_irq,
    stm32_pin_irq_enable,
};

int rt_hw_pin_init(void)
{
    int result;
    result = rt_device_pin_register("pin", &_stm32_pin_ops, RT_NULL);
    return result;
}
INIT_BOARD_EXPORT(rt_hw_pin_init);

static void pin_irq_hdr(uint16_t GPIO_Pin)
{
    int irqno = 0;
    for(irqno = 0; irqno < 16; irqno ++)
    {
        if((0x01 << irqno) == GPIO_Pin)
        {
            break;
        }
    }
    if(irqno == 16)return;
    if (pin_irq_hdr_tab[irqno].hdr)
    {
        pin_irq_hdr_tab[irqno].hdr(pin_irq_hdr_tab[irqno].args);
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    pin_irq_hdr(GPIO_Pin);
}

void EXTI0_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
    rt_interrupt_leave();
}

void EXTI1_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
    rt_interrupt_leave();
}

void EXTI2_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_2);
    rt_interrupt_leave();
}

void EXTI3_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    rt_interrupt_leave();
}

void EXTI4_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
    rt_interrupt_leave();
}

void EXTI9_5_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_6);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_7);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_8);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_9);
    rt_interrupt_leave();
}

void EXTI15_10_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_10);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_11);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_14);
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
    rt_interrupt_leave();
}
#endif
'''

default_pin_string = '    __STM32_PIN_DEFAULT,\n'

def get_pin_string(index, port, no):
    return '    __STM32_PIN(' + index +', ' + port + ', ' + no+ '),\n'


def generate_gpio_driver_by_cube_xml(xml_path, save_path):
    Document = xml.dom.minidom.parse(xml_path)
    Elements = Document.getElementsByTagName('Pin')
    pattern = 'P([A-Z])([0-9]{1,})'
    with open(save_path ,'w') as f:
        mcu = Document.getElementsByTagName('Mcu')
        f.write(get_header_info(mcu[0].getAttribute('RefName')))
        f.write(gpio_template_header_string)
        f.write(default_pin_string)
        for pin in Elements:
            if pin.getAttribute('Type') != 'I/O':
                f.write(default_pin_string)
            else:
                match = re.match(pattern, pin.getAttribute('Name'))
                group = match.group(1)
                no = match.group(2)
                f.write(get_pin_string(pin.getAttribute('Position'), group, no))
        f.write(gpio_template_end_string)

import sys

if __name__ == '__main__':
    generate_gpio_driver_by_cube_xml(sys.argv[1],'drv_gpio.c')
