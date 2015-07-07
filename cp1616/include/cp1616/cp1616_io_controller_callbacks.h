/*********************************************************************************************//**
* @file cp1616_io_controller_callbacks.h
*
* Callbacks declarations - required by existing IO Base library callback interface
*
* Copyright {2015} {Frantisek Durovsky}
*
*  Licensed under the Apache License, Version 2.0 (the "License");
*  you may not use this file except in compliance with the License.
*  You may obtain a copy of the License at

*      http://www.apache.org/licenses/LICENSE-2.0

*  Unless required by applicable law or agreed to in writing, software
*  distributed under the License is distributed on an "AS IS" BASIS,
*  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*  See the License for the specific language governing permissions and
*  limitations under the License.

* *********************************************************************************************/
#ifndef CP1616_IO_CONTROLLER_CALLBACKS_H
#define CP1616_IO_CONTROLLER_CALLBACKS_H

namespace cp1616
{
namespace pnio_controller_callbacks
{

  /**
   * \brief IO Controller local mode has changed
   *
   * \param [in]  *p_Cbf_Prm        pointer to PNIO_CBE_PRM struct
   */
          void modeChangeIndication(PNIO_CBE_PRM *p_cbf_prm);

  /**
   * \brief Signals connection status to the IO device
   *
   * \param [in]  *p_Cbf_Prm        pointer to PNIO_CBE_PRM struct
   */
          void deviceActivation(PNIO_CBE_PRM *p_cbf_prm);

  /**
   * \brief Alarm Indication callback
   *
   * \param [in]  *p_Cbf_Prm        pointer to PNIO_CBE_PRM struct
   */
          void alarmIndication(PNIO_CBE_PRM *p_cbf_prm);

  /**
   * \brief mandatory callback for PNIO_open_controller
   *
   * \param [in]  *p_Cbf_Prm        pointer to PNIO_CBE_PRM struct
   */
          void dsReadConf(PNIO_CBE_PRM *p_cbf_prm);

  /**
   * \brief mandatory callback for PNIO_open_controller
   *
   * \param [in]  *p_Cbf_Prm        pointer to PNIO_CBE_PRM struct
   */
          void dsWriteConf(PNIO_CBE_PRM *p_cbf_prm);
  
} //pnio_controller_callbacks
} //cp1616

#endif //CP1616_IO_CONTROLLER_CALLBACKS_H
