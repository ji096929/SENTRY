/*------------------------------------------------------------------------------
 * MDK Middleware - Component ::USB:Device
 * Copyright (c) 2004-2013 ARM Germany GmbH. All rights reserved.
 *------------------------------------------------------------------------------
 * Name:    USBD_Config_HID_1.h
 * Purpose: USB Device Human Interface Device class (HID) Configuration
 * Rev.:    V5.10
 *----------------------------------------------------------------------------*/

//-------- <<< Use Configuration Wizard in Context Menu >>> --------------------

// <h>USB Device: Human Interface Device class (HID) 1
//   <o>Assign Device Class to USB Device # <0-3>
//   <i>Select USB Device that is used for this Device Class instance
#define USBD_HID1_DEV                   1

//   <h>Interrupt Endpoint Settings
//   <i>By default, the settings match the first USB Class instance in a USB Device.
//   <i>Endpoint conflicts are flagged by compile-time error messages.
//
//     <o.0..3>Interrupt IN Endpoint Number
//               <1=>1   <2=>2   <3=>3   <4=>4   <5=>5   <6=>6   <7=>7
//       <8=>8   <9=>9   <10=>10 <11=>11 <12=>12 <13=>13 <14=>14 <15=>15
#define USBD_HID1_EP_INT_IN             1

//     <o.0..3>Interrupt OUT Endpoint Number
//       <i>When OUT Endpoint is set to "Not used" the USB Host uses 
//       <i>the Control Endpoint 0 for Out Reports.
//       <0=>Not used
//               <1=>1   <2=>2   <3=>3   <4=>4   <5=>5   <6=>6   <7=>7
//       <8=>8   <9=>9   <10=>10 <11=>11 <12=>12 <13=>13 <14=>14 <15=>15
#define USBD_HID1_EP_INT_OUT            1


//     <h>Endpoint Settings
//       <i>Parameters are used to create USB Descriptors, HID Device Descriptor
//       <i>and for memory allocation in the USB component.
//
//       <h>Full/Low-speed (High-speed disabled)
//       <i>Parameters apply when High-speed is disabled in USBD_Config_1.c
//         <o.0..6>Maximum Endpoint Packet Size (in bytes) <0-64>
//         <i>Specifies the physical packet size used for information exchange.
//         <i>Maximum value is 64.
#define USBD_HID1_WMAXPACKETSIZE        4

//         <o.0..7>Endpoint polling Interval (in ms) <1-255>
//         <i>Specifies the frequency of requests initiated by USB Host for 
//         <i>function USBD_HIDn_GetReport.
#define USBD_HID1_BINTERVAL             16

//       </h>

//       <h>High-speed
//       <i>Parameters apply when High-speed is enabled in USBD_Config_1.c
//
//         <o.0..10>Maximum Endpoint Packet Size (in bytes) <0-1024>
//         <i>Specifies the physical packet size used for information exchange.
//         <i>Maximum value is 1024.
//         <o.11..12>Additional transactions per microframe 
//         <i>Additional transactions improve communication performance.
//           <0=>None <1=>1 additional <2=>2 additional
#define USBD_HID1_HS_WMAXPACKETSIZE     4

//         <o.0..4>Endpoint polling Interval (in 125 us intervals) 
//         <i>Specifies the frequency of requests initiated by USB Host for 
//         <i>function USBD_HIDn_GetReport.
//           <1=>    1 <2=>    2 <3=>     4 <4=>     8
//           <5=>   16 <6=>   32 <7=>    64 <8=>   128
//           <9=>  256 <10=> 512 <11=> 1024 <12=> 2048
//           <13=>4096 <14=>8192 <15=>16384 <16=>32768
#define USBD_HID1_HS_BINTERVAL          5

//       </h>
//     </h>
//   </h>

//   <h>Human Interface Device Class Settings
//   <i>Parameters are used to create USB Descriptors, USB HID Report Descriptor 
//   <i>and for memory allocation in the USB component.
//
//     <s.126>HID Interface String
#define USBD_HID1_STR_DESC              L"USB_HID1"

//     <o.0..4>Number of Input Reports <1-32>
//     <i>Configures max 'rid' value for USBD_HID1_GetReport and USBD_HID_GetReportTrigger
#define USBD_HID1_IN_REPORT_NUM         1

//     <o.0..4>Number of Output Reports <1-32>
//     <i>Configures max 'rid' value for USBD_HID1_SetReport
#define USBD_HID1_OUT_REPORT_NUM        1

//     <o.0..15>Maximum Input Report Size (in bytes) <1-65535>
//     <i>Allocates memory and configures 'len' value for USBD_HID1_GetReport 
//     <i>and USBD_HID_GetReportTrigger 
#define USBD_HID1_IN_REPORT_MAX_SZ      1

//     <o.0..15>Maximum Output Report Size (in bytes) <1-65535>
//     <i>Allocates memory and configures 'len' value for USBD_HID1_SetReport 
//     <i>when rtype=HID_REPORT_OUTPUT.
#define USBD_HID1_OUT_REPORT_MAX_SZ     1

//     <o.0..15>Maximum Feature Report Size (in bytes) <1-65535>
//     <i>Allocates memory and configures 'len' value for USBD_HID1_SetReport 
//     <i>when rtype=HID_REPORT_FEATURE
#define USBD_HID1_FEAT_REPORT_MAX_SZ    1

//   </h>

//   <h>OS Resources Settings
//   <i>These settings are used to optimize usage of OS resources.
//     <o>Human Interface Device Class Thread Stack Size <64-65536>
#define USBD_HID1_THREAD_STACK_SIZE     512

//        Human Interface Device Class Thread Priority
#define USBD_HID1_THREAD_PRIORITY       osPriorityAboveNormal

//   </h>
// </h>
