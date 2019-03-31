#if defined(TARGET_NUCLEO_F401RE)||defined(TARGET_NUCLEO_F411RE)||defined(TARGET_NUCLEO_F446RE)
#include "USBHALHost.h"

// usbh_conf.c
HCD_HandleTypeDef hhcd_USB_OTG_FS;

static void HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd);

// stm32f4xx_it.c
extern "C" {
void OTG_FS_IRQHandler(void)
{
  HAL_NVIC_ClearPendingIRQ(OTG_FS_IRQn);
  HAL_HCD_IRQHandler(&hhcd_USB_OTG_FS);
}
}

// stm32f4xx_hal_hcd.c
static void HCD_HC_IN_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum);
static void HCD_HC_OUT_IRQHandler(HCD_HandleTypeDef *hhcd, uint8_t chnum); 
static void HCD_RXQLVL_IRQHandler(HCD_HandleTypeDef *hhcd);
static void HCD_Port_IRQHandler(HCD_HandleTypeDef *hhcd);

/**
  * @brief  This function handles HCD interrupt request.
  * @param  hhcd: HCD handle
  * @retval none
  */
static void HAL_HCD_IRQHandler(HCD_HandleTypeDef *hhcd)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;
  uint32_t i = 0 , interrupt = 0;
  
  /* ensure that we are in device mode */
  if (USB_GetMode(hhcd->Instance) == USB_OTG_MODE_HOST)
  {
    /* avoid spurious interrupt */
    if(__HAL_HCD_IS_INVALID_INTERRUPT(hhcd)) 
    {
      return;
    }
    
    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT))
    {
     /* incorrect mode, acknowledge the interrupt */
      __HAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GINTSTS_PXFR_INCOMPISOOUT);
    }
    
    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_IISOIXFR))
    {
     /* incorrect mode, acknowledge the interrupt */
      __HAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GINTSTS_IISOIXFR);
    }

    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_PTXFE))
    {
     /* incorrect mode, acknowledge the interrupt */
      __HAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GINTSTS_PTXFE);
    }   
    
    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_MMIS))
    {
     /* incorrect mode, acknowledge the interrupt */
      __HAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GINTSTS_MMIS);
    }     
    
    /* Handle Host Disconnect Interrupts */
    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_DISCINT))
    {
      
      /* Cleanup HPRT */
      USBx_HPRT0 &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET |\
        USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG );
       
      /* Handle Host Port Interrupts */
      HAL_HCD_Disconnect_Callback(hhcd);
       USB_InitFSLSPClkSel(hhcd->Instance ,HCFG_48_MHZ );
      __HAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GINTSTS_DISCINT);
    }
    
    /* Handle Host Port Interrupts */
    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_HPRTINT))
    {
      HCD_Port_IRQHandler (hhcd);
    }
    
    /* Handle Host SOF Interrupts */
    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_SOF))
    {
      HAL_HCD_SOF_Callback(hhcd);
      __HAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GINTSTS_SOF);
    }
          
    /* Handle Host channel Interrupts */
    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_HCINT))
    {
      
      interrupt = USB_HC_ReadInterrupt(hhcd->Instance);
      for (i = 0; i < hhcd->Init.Host_channels ; i++)
      {
        if (interrupt & (1 << i))
        {
          if ((USBx_HC(i)->HCCHAR) &  USB_OTG_HCCHAR_EPDIR)
          {
            HCD_HC_IN_IRQHandler (hhcd, i);
          }
          else
          {
            HCD_HC_OUT_IRQHandler (hhcd, i);
          }
        }
      }
      __HAL_HCD_CLEAR_FLAG(hhcd, USB_OTG_GINTSTS_HCINT);
    } 
    
        /* Handle Rx Queue Level Interrupts */
    if(__HAL_HCD_GET_FLAG(hhcd, USB_OTG_GINTSTS_RXFLVL))
    {
      USB_MASK_INTERRUPT(hhcd->Instance, USB_OTG_GINTSTS_RXFLVL);
      
      HCD_RXQLVL_IRQHandler (hhcd);
      
      USB_UNMASK_INTERRUPT(hhcd->Instance, USB_OTG_GINTSTS_RXFLVL);
    }
    
  }
}
/**
  * @brief  This function handles Host Channel IN interrupt requests.
  * @param  hhcd: HCD handle
  * @param  chnum : Channel number
  *         This parameter can be a value from 1 to 15
  * @retval none
  */
static void HCD_HC_IN_IRQHandler   (HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;
    
  if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_AHBERR)
  {
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_AHBERR);
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum);
  }  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_ACK)
  {
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_ACK);
  }
  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_STALL)  
  {
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum);
    hhcd->hc[chnum].state = HC_STALL;
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_NAK);
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_STALL);    
    USB_HC_Halt(hhcd->Instance, chnum);    
  }
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_DTERR)
  {
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum);
    USB_HC_Halt(hhcd->Instance, chnum);  
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_NAK);    
    hhcd->hc[chnum].state = HC_DATATGLERR;
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_DTERR);
  }    
  
  if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_FRMOR)
  {
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
    USB_HC_Halt(hhcd->Instance, chnum);  
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_FRMOR);
  }
  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_XFRC)
  {
    
    if (hhcd->Init.dma_enable)
    {
      hhcd->hc[chnum].xfer_count = hhcd->hc[chnum].xfer_len - \
                               (USBx_HC(chnum)->HCTSIZ & USB_OTG_HCTSIZ_XFRSIZ);
    }
    
    hhcd->hc[chnum].state = HC_XFRC;
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_XFRC);
    
    
    if ((hhcd->hc[chnum].ep_type == EP_TYPE_CTRL)||
        (hhcd->hc[chnum].ep_type == EP_TYPE_BULK))
    {
      __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
      USB_HC_Halt(hhcd->Instance, chnum); 
      __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_NAK);
      
    }
    else if(hhcd->hc[chnum].ep_type == EP_TYPE_INTR)
    {
      USBx_HC(chnum)->HCCHAR |= USB_OTG_HCCHAR_ODDFRM;
      __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
      USB_HC_Halt(hhcd->Instance, chnum); 
      __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_NAK);
    }
    else if(hhcd->hc[chnum].ep_type == EP_TYPE_ISOC)
    {
      USBx_HC(chnum)->HCCHAR |= USB_OTG_HCCHAR_ODDFRM;
      hhcd->hc[chnum].urb_state = URB_DONE; 
      HAL_HCD_HC_NotifyURBChange_Callback(hhcd, chnum, hhcd->hc[chnum].urb_state);
    }
    hhcd->hc[chnum].toggle_in ^= 1;
    
  }
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_CHH)
  {
    __HAL_HCD_MASK_HALT_HC_INT(chnum); 
    
    if(hhcd->hc[chnum].state == HC_XFRC)
    {
      hhcd->hc[chnum].urb_state  = URB_DONE;      
    }

    else if (hhcd->hc[chnum].state == HC_NAK)
    {
      hhcd->hc[chnum].urb_state  = URB_DONE;
    }
    
    else if (hhcd->hc[chnum].state == HC_STALL) 
    {
      hhcd->hc[chnum].urb_state  = URB_STALL;
    }   
    
    else if (hhcd->hc[chnum].state == HC_XACTERR)
    {
        hhcd->hc[chnum].urb_state = URB_NOTREADY;
    }

    else if (hhcd->hc[chnum].state == HC_DATATGLERR)
    {
        hhcd->hc[chnum].urb_state = URB_ERROR;
    }

    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_CHH);
    HAL_HCD_HC_NotifyURBChange_Callback(hhcd, chnum, hhcd->hc[chnum].urb_state);
  }  
  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_TXERR)
  {
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
     hhcd->hc[chnum].state = HC_XACTERR;
     USB_HC_Halt(hhcd->Instance, chnum);     
     __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_TXERR);
  }
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_NAK)
  {  
    if(hhcd->hc[chnum].ep_type == EP_TYPE_INTR)
    {
      __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
      USB_HC_Halt(hhcd->Instance, chnum);  
    }
    else if (hhcd->hc[chnum].ep_type == EP_TYPE_CTRL)
    {
      /* re-activate the channel  */
      USBx_HC(chnum)->HCCHAR &= ~USB_OTG_HCCHAR_CHDIS;         
      USBx_HC(chnum)->HCCHAR |= USB_OTG_HCCHAR_CHENA;
   
    }
    
    else if (hhcd->hc[chnum].ep_type == EP_TYPE_BULK)
    {
      __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
      USB_HC_Halt(hhcd->Instance, chnum); 
    }

    hhcd->hc[chnum].state = HC_NAK;
     __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_NAK);
  }
}

/**
  * @brief  This function handles Host Channel OUT interrupt requests.
  * @param  hhcd: HCD handle
  * @param  chnum : Channel number
  *         This parameter can be a value from 1 to 15
  * @retval none
  */
static void HCD_HC_OUT_IRQHandler  (HCD_HandleTypeDef *hhcd, uint8_t chnum)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;
  
  if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_AHBERR)
  {
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_AHBERR);
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum);
  }  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_ACK)
  {
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_ACK);
    
    if( hhcd->hc[chnum].do_ping == 1)
    {
      hhcd->hc[chnum].state = HC_NYET;     
      __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
      USB_HC_Halt(hhcd->Instance, chnum); 
      hhcd->hc[chnum].urb_state  = URB_NOTREADY;
    }
  }
  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_NYET)
  {
    hhcd->hc[chnum].state = HC_NYET;
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
    USB_HC_Halt(hhcd->Instance, chnum);      
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_NYET);
    
  }  
  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_FRMOR)
  {
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
    USB_HC_Halt(hhcd->Instance, chnum);  
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_FRMOR);
  }
  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_XFRC)
  {
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum);
    USB_HC_Halt(hhcd->Instance, chnum);   
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_XFRC);
    hhcd->hc[chnum].state = HC_XFRC;

  }  

  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_STALL)  
  {
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_STALL);  
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum);
    USB_HC_Halt(hhcd->Instance, chnum);   
    hhcd->hc[chnum].state = HC_STALL;    
  }

  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_NAK)
  {  
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
    USB_HC_Halt(hhcd->Instance, chnum);   
    hhcd->hc[chnum].state = HC_NAK;
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_NAK);
  }

  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_TXERR)
  {
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
    USB_HC_Halt(hhcd->Instance, chnum);      
    hhcd->hc[chnum].state = HC_XACTERR;  
     __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_TXERR);
  }
  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_DTERR)
  {
    __HAL_HCD_UNMASK_HALT_HC_INT(chnum); 
    USB_HC_Halt(hhcd->Instance, chnum);      
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_NAK);
    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_DTERR);    
    hhcd->hc[chnum].state = HC_DATATGLERR;
  }
  
  
  else if ((USBx_HC(chnum)->HCINT) &  USB_OTG_HCINT_CHH)
  {
    __HAL_HCD_MASK_HALT_HC_INT(chnum); 
    
    if(hhcd->hc[chnum].state == HC_XFRC)
    {
      hhcd->hc[chnum].urb_state  = URB_DONE;
      if (hhcd->hc[chnum].ep_type == EP_TYPE_BULK)
      {
        hhcd->hc[chnum].toggle_out ^= 1; 
      }      
    }
    else if (hhcd->hc[chnum].state == HC_NAK) 
    {
      hhcd->hc[chnum].urb_state  = URB_NOTREADY;
    }  
    
    else if (hhcd->hc[chnum].state == HC_NYET) 
    {
      hhcd->hc[chnum].urb_state  = URB_NOTREADY;
      hhcd->hc[chnum].do_ping = 0;
    }   
    
    else if (hhcd->hc[chnum].state == HC_STALL) 
    {
      hhcd->hc[chnum].urb_state  = URB_STALL;
    } 
    
    else if(hhcd->hc[chnum].state == HC_XACTERR)
    {
      hhcd->hc[chnum].urb_state = URB_NOTREADY;
    }
    
    else if (hhcd->hc[chnum].state == HC_DATATGLERR)
    {
      hhcd->hc[chnum].urb_state = URB_ERROR;
    }

    __HAL_HCD_CLEAR_HC_INT(chnum, USB_OTG_HCINT_CHH);
    HAL_HCD_HC_NotifyURBChange_Callback(hhcd, chnum, hhcd->hc[chnum].urb_state);  
  }
} 

/**
  * @brief  This function handles Rx Queue Level interrupt requests.
  * @param  hhcd: HCD handle
  * @retval none
  */
static void HCD_RXQLVL_IRQHandler  (HCD_HandleTypeDef *hhcd)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;  
  uint8_t                       channelnum =0;  
  uint32_t                      pktsts;
  uint32_t                      pktcnt; 
  uint32_t                      temp = 0;
  
  temp = hhcd->Instance->GRXSTSP ;
  channelnum = temp &  USB_OTG_GRXSTSP_EPNUM;  
  pktsts = (temp &  USB_OTG_GRXSTSP_PKTSTS) >> 17;
  pktcnt = (temp &  USB_OTG_GRXSTSP_BCNT) >> 4;
    
  switch (pktsts)
  {
  case GRXSTS_PKTSTS_IN:
    /* Read the data into the host buffer. */
    if ((pktcnt > 0) && (hhcd->hc[channelnum].xfer_buff != (void  *)0))
    {  
      
      USB_ReadPacket(hhcd->Instance, hhcd->hc[channelnum].xfer_buff, pktcnt);
     
      /*manage multiple Xfer */
      hhcd->hc[channelnum].xfer_buff += pktcnt;           
      hhcd->hc[channelnum].xfer_count  += pktcnt;
        
      if((USBx_HC(channelnum)->HCTSIZ & USB_OTG_HCTSIZ_PKTCNT) > 0)
      {
        /* re-activate the channel when more packets are expected */
        USBx_HC(channelnum)->HCCHAR &= ~USB_OTG_HCCHAR_CHDIS; 
        USBx_HC(channelnum)->HCCHAR |= USB_OTG_HCCHAR_CHENA;
        hhcd->hc[channelnum].toggle_in ^= 1;
      }
    }
    break;

  case GRXSTS_PKTSTS_DATA_TOGGLE_ERR:
    break;
  case GRXSTS_PKTSTS_IN_XFER_COMP:
  case GRXSTS_PKTSTS_CH_HALTED:
  default:
    break;
  }
}

/**
  * @brief  This function handles Host Port interrupt requests.
  * @param  hhcd: HCD handle
  * @retval none
  */
static void HCD_Port_IRQHandler  (HCD_HandleTypeDef *hhcd)
{
  USB_OTG_GlobalTypeDef *USBx = hhcd->Instance;  
  __IO uint32_t hprt0, hprt0_dup;
  
  /* Handle Host Port Interrupts */
  hprt0 = USBx_HPRT0;
  hprt0_dup = USBx_HPRT0;
  
  hprt0_dup &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET |\
                 USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG );
  
  /* Check wether Port Connect Detected */
  if((hprt0 & USB_OTG_HPRT_PCDET) == USB_OTG_HPRT_PCDET)
  {  
    if((hprt0 & USB_OTG_HPRT_PCSTS) == USB_OTG_HPRT_PCSTS)
    {
      USB_MASK_INTERRUPT(hhcd->Instance, USB_OTG_GINTSTS_DISCINT);
      HAL_HCD_Connect_Callback(hhcd);
    }
    hprt0_dup  |= USB_OTG_HPRT_PCDET;
    
  }
  
  /* Check whether Port Enable Changed */
  if((hprt0 & USB_OTG_HPRT_PENCHNG) == USB_OTG_HPRT_PENCHNG)
  {
    hprt0_dup |= USB_OTG_HPRT_PENCHNG;
    
    if((hprt0 & USB_OTG_HPRT_PENA) == USB_OTG_HPRT_PENA)
    {    
      if(hhcd->Init.phy_itface  == USB_OTG_EMBEDDED_PHY)
      {
        if ((hprt0 & USB_OTG_HPRT_PSPD) == (HPRT0_PRTSPD_LOW_SPEED << 17))
        {
          USB_InitFSLSPClkSel(hhcd->Instance ,HCFG_6_MHZ );
        }
        else
        {
          USB_InitFSLSPClkSel(hhcd->Instance ,HCFG_48_MHZ );
        }
      }
      else
      {
        if(hhcd->Init.speed == HCD_SPEED_FULL)
        {
          USBx_HOST->HFIR = (uint32_t)60000;
        }
      }
      HAL_HCD_Connect_Callback(hhcd);
      
      if(hhcd->Init.speed == HCD_SPEED_HIGH)
      {
        USB_UNMASK_INTERRUPT(hhcd->Instance, USB_OTG_GINTSTS_DISCINT); 
      }
    }
    else
    {
      /* Cleanup HPRT */
      USBx_HPRT0 &= ~(USB_OTG_HPRT_PENA | USB_OTG_HPRT_PCDET |\
        USB_OTG_HPRT_PENCHNG | USB_OTG_HPRT_POCCHNG );
      
      USB_UNMASK_INTERRUPT(hhcd->Instance, USB_OTG_GINTSTS_DISCINT); 
    }    
  }
  
  /* Check For an overcurrent */
  if((hprt0 & USB_OTG_HPRT_POCCHNG) == USB_OTG_HPRT_POCCHNG)
  {
    hprt0_dup |= USB_OTG_HPRT_POCCHNG;
  }

  /* Clear Port Interrupts */
  USBx_HPRT0 = hprt0_dup;
}

#endif


