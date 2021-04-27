#include "hdlc.h"
#include "main.h"


extern UART_HandleTypeDef huart1;

interfaceHDLC_TX masterTX;
interfaceHDLC_RX masterRX;

//-------------------------------------------------------------
// процедура установки байта в буфер передачи с заменой  
//-------------------------------------------------------------
unsigned add_data_tx_buff (unsigned char data, unsigned int index)
{
	if(data == 0x7e)
	{  
		masterTX.master_buff_tx[index] = 0x7d;
		masterTX.master_buff_tx[index+1] = 0x5e; // если входной байт 0x7e меняем на два 0x7d 0x5e
		return 2;
	}
	else
	{
		if(data == 0x7d)
		{
			masterTX.master_buff_tx[index] = 0x7d;
			masterTX.master_buff_tx[index+1] = 0x5d;// если входной байт 0x7d меняем на два 0x7d 0x5d
			return 2;
		}
		else
		{
			masterTX.master_buff_tx[index] = data; // иначе записываем как есть
			return 1;
		}			
	}		
}

//-------------------------------------------------------------
// процедура подготовки данных для передачи состояния сенсора в буфер мастера
//-------------------------------------------------------------
unsigned int send_Master_HDLC (struct_HDLC_Header MasterHDLC,char Command, char* data, int count_data)
{

	unsigned int count_master_TX;				      // индекс байта, куда кладем данные
	uint16_t crcSend = CRC16((uint8_t*)data,count_data);

	count_master_TX = 0;
	masterTX.master_buff_tx[count_master_TX++] = 0x7e;	              // маркер начала и конца посылки

	count_master_TX += add_data_tx_buff(MasterHDLC.ADR_DST,count_master_TX); // передаем адрес получателя посылки
	count_master_TX += add_data_tx_buff(MasterHDLC.ADR_SRC,count_master_TX); // передаем адрес отправителя
	count_master_TX += add_data_tx_buff(Command,count_master_TX); // передаем адрес отправителя
	
	for (unsigned int index = 0; index < count_data; index++)
	{
		count_master_TX += add_data_tx_buff(data[index],count_master_TX); // данные сенсора
	}
	// дописать процедуру подсчета CRC
	count_master_TX += add_data_tx_buff((uint8_t)(crcSend & 0x00FF),count_master_TX); // данные сенсора
	count_master_TX += add_data_tx_buff((uint8_t)((crcSend & 0xFF00)>>8),count_master_TX); // данные сенсора
	
	masterTX.master_buff_tx[count_master_TX++] = 0x7e;	              // маркер начала и конца посылки
	
	HAL_UART_Transmit(&huart1, masterTX.master_buff_tx, count_master_TX, 4000);	
	return count_master_TX;
}

//-------------------------------------------------------------
//  обработка принятого байта
//-------------------------------------------------------------
void recHDLC (uint8_t data)
{
   uint16_t CRC_in, CRC_calk;
	 switch (data)
   {
      case 0x7E:

          if (masterRX.RX_StartFlag == 1)
          {
              masterRX.RX_7EFlag = 0;
              // приняли всю посылку, разбираем
              if (masterRX.count_RX > 3)
              {
                  CRC_in = (masterRX.master_buff_rx[masterRX.count_RX - 2] + masterRX.master_buff_rx[masterRX.count_RX - 1] * 256);
                  CRC_calk = CRC16(masterRX.master_buff_rx,masterRX.count_RX-2);
									CRC_in = 0;
									CRC_calk = 0;
                  if (CRC_in == CRC_calk)
                  {
                      masterRX.ADR_DSC = masterRX.master_buff_rx[0];
                      masterRX.ADR_SRC = masterRX.master_buff_rx[1];
                      masterRX.Command = masterRX.master_buff_rx[2];
											masterRX.argCount = masterRX.count_RX - 5;
                      for (int i = 0; i < masterRX.argCount; i++)
                        masterRX.arg[i] = masterRX.master_buff_rx[i + 3];
                  }
									masterRX.RX_StartFlag = 0;
									masterRX.RX_7DFlag = 0;
									masterRX.RX_7EFlag = 0;
									masterRX.count_RX = 0;
									 	

               }

            }
            else
            {
                masterRX.RX_StartFlag = 1;
                masterRX.count_RX = 0;


            }
            break;

        case 0x7D:
            masterRX.RX_7DFlag = 1;
            break;

        case 0x5E:

            if (masterRX.RX_StartFlag == 1)
            {
                if (masterRX.RX_7DFlag == 1)
                {
                    masterRX.master_buff_rx[masterRX.count_RX++] = 0x7E;
									  masterRX.RX_7DFlag = 0;
                }
                else
                {
                    masterRX.master_buff_rx[masterRX.count_RX++] = 0x5E;
                }
            }
            break;
        case 0x5D:

            if (masterRX.RX_StartFlag == 1)
            {
                if (masterRX.RX_7DFlag == 1)
                {
                    masterRX.master_buff_rx[masterRX.count_RX++] = 0x7D;
									  masterRX.RX_7DFlag = 0;
                }
                else
                {
                    masterRX.master_buff_rx[masterRX.count_RX++] = 0x5D;
                }
            }
            break;

        default:

            if(masterRX.RX_StartFlag == 1)
                masterRX.master_buff_rx[masterRX.count_RX++] = data;

            if (masterRX.count_RX > MAX_HDLC_RX_BUFFER)
            {
                masterRX.RX_StartFlag = 0;
                masterRX.RX_7DFlag = 0;
                masterRX.count_RX = 0;
            }
            break;


    }
}

//-------------------------------------------------------------
// подсчет CRC16
//-------------------------------------------------------------
uint16_t CRC16(uint8_t * data, int count)
{
		uint16_t crc = 0xFFFF;
		int i, index;

		for (index = 0; index < count; index++)
		{
				crc ^= (data[index] << 8);

				for (i = 0; i < 8; i++)
						crc = (crc & 0x8000) == 0x8000 ? ((crc << 1) ^ 0x1021) : (crc << 1);
		}

		return crc;
}
