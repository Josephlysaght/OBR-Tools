% Script for creating car struct for testing purposes

chassisParameters = 'ChassisParameters.xlsx';
tyreCoefficients = 'Tyre_Hoosier_6_18_10_LC0.xlsx';
powertrainParameters = 'KTM510Powertrain.xlsx';
aeroParameters = 'AeroParameters.xlsx';
kinData = 'LTS_kin_input_sheet.xlsx';

Car = createCarStruct(chassisParameters, tyreCoefficients, powertrainParameters, aeroParameters, kinData);

clear chassisParameters tyreCoefficients powertrainParameters aeroParameters kinData