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

/**
   * \brief IO Controller local mode has changed
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          void PNIOCbfForModeChangeIndication(PNIO_CBE_PRM *pCbfPrm);

  /**
   * \brief Signals connection status to IO device
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          void PNIOCbfForDeviceActivation(PNIO_CBE_PRM *pCbfPrm);

  /**
   * \brief Alarm Indication
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          void PNIOCbfForAlarmIndication(PNIO_CBE_PRM *pCbfPrm);

  /**
   * \brief mandatory callback for PNIO_open_controller
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          void PNIOCbfForDsReadConf(PNIO_CBE_PRM *pCbfPrm);

  /**
   * \brief mandatory callback for PNIO_open_controller
   *
   * \param pointer to PNIO_CBE_PRM struct
   */
          void PNIOCbfForDsWriteConf(PNIO_CBE_PRM *pCbfPrm);

#endif //CP1616_IO_CONTROLLER_CALLBACKS_H
