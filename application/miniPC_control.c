#include "main.h"
#include "miniPC_control.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;

// DMA ˫������
uint8_t usart6_rx_buf[2][24]; 

float miniPC_yaw = 0, miniPC_pitch = 0,fire = 1;
static float yaw = 0, pitch = 0;

void miniPC_control_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //ʹ��DMA���ڽ���
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    //ʹ�ܿ����ж�
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }
    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //�ڴ滺����1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //�ڴ滺����2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //���ݳ���
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //ʹ��˫������
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);
    //ʹ��DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_12, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_11, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_RESET);

}

void USART6_IRQHandler(void)  
{
    volatile uint8_t receive;

    // �����ж�
    if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE))
    {
        receive = huart6.Instance->DR;
        // �����Ҫ�������ڴ˴������ֽڼ���������
    }

    // �����ж�
    else if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
    {
        // �ؼ������ IDLE �жϱ�־λ��ԭ�������ʹ���� __HAL_UART_CLEAR_PEFLAG��
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);

        static uint16_t this_time_rx_len = 0;

        if ((hdma_usart6_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            // ��ǰʹ�õ��� Memory 0
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = 24u - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = 24u;
            hdma_usart6_rx.Instance->CR |= DMA_SxCR_CT;  // �л��� Memory 1
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == 12u)
            {
                memcpy(&yaw,   &usart6_rx_buf[0][8], 4);
                memcpy(&pitch, &usart6_rx_buf[0][4], 4);
                memcpy(&fire,  &usart6_rx_buf[0][0], 4);
                HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);

                if (yaw <= 3.14f)   miniPC_yaw   += yaw;
                if (pitch <= 3.14f) miniPC_pitch += pitch;
            }
        }
        else
        {
            // ��ǰʹ�õ��� Memory 1
            __HAL_DMA_DISABLE(&hdma_usart6_rx);
            this_time_rx_len = 24u - hdma_usart6_rx.Instance->NDTR;
            hdma_usart6_rx.Instance->NDTR = 24u;
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);  // �л��� Memory 0
            __HAL_DMA_ENABLE(&hdma_usart6_rx);

            if (this_time_rx_len == 12u)
            {
                memcpy(&yaw,   &usart6_rx_buf[1][0], 4);
                memcpy(&pitch, &usart6_rx_buf[1][4], 4);
                memcpy(&fire,  &usart6_rx_buf[1][8], 4);
                HAL_GPIO_TogglePin(GPIOH, GPIO_PIN_10);

                if (yaw <= 3.14f)   miniPC_yaw   += yaw;
                if (pitch <= 3.14f) miniPC_pitch += pitch;
            }
        }
    }
}
