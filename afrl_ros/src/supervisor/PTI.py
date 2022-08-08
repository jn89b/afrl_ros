from mavros import param
from afrl_configs import pti_config, attitude_constraints


class PTIParamVerifier():
    """Class FTIParamVerifier makes sure the parameters submitted are within the limtis
    of the system"""
    def set_injection_vals(self, inject_param: str, inject_param_val:int, 
                            inject_setting:str):
        """set injection values based on parameter location"""
        if inject_param == "FTI_INJXN_POINT":
            level_vals = pti_config.FTI_INJXN_POINT[str(inject_param_val)]
            level_index = pti_config.input_control_index[inject_setting]
            param.param_set(inject_param, int(inject_param_val))
            param.param_set("FTI_FS_AMP_BEGIN", float(level_vals[level_index]))
            param.param_set("FTI_FS_AMP_END", float(level_vals[level_index]))
            print("set values", )

    def set_pti_param(self, pti_param: str , pti_param_val):
        """checks if pti param exists, not duplicate,
         values are good, and then sets value"""

        if self.check_pti_param_exist(pti_param) == False:
            print("doesn't exist", pti_param)
            return 

        if self.check_dup_param_val(pti_param, pti_param_val) == True:
            print("duplicate value", pti_param, " ", pti_param_val)
            return 

        param.param_set(pti_param, pti_param_val)
        print("set param ", pti_param, " to ", pti_param_val)
        # pti_param_range = pti_config.PTI_CONFIG[pti_param]
 
        # # set amplitudes based on injection point
        # if pti_param == "FTI_INJXN_POINT":
        #     print("injection point")
        # # i 
        # # if pti_param == "FTI_FS_AMP_BEGIN" or pti_param == "FTI_FS_AMP_END":
        #     if pti_param_val in pti_config.input_control_index:
        #         setting_vals = pti_config.FTI_INJXN_POINT[pti_param_val]                
        #         # if setting_vals == None:
        #         #     return 
                
        #         control_index = pti_config.input_control_index[pti_param_val]
        #         print("value is ", setting_vals[control_index])
        #         param.param_set(pti_param, setting_vals[control_index])
        #         return 
            
            # else:
            #     print("nope")
            #     return 

        # #set frequency
        # if pti_param_val<= pti_param_range[1] and pti_param_val>=pti_param_range[0]:
        #     param.param_set(pti_param, pti_param_val)
        #     print("Set PTI", pti_param, "to", pti_param_val)
        #     return 
        # else:
        #     print("value is out of range", pti_param_val, 
        #     "range of values are ", pti_param_range)
        #     return 

    def check_dup_param_val(self, pti_param: str, pti_param_val) -> bool:
        """check if parameter value is dup/same"""
        pti_val = self.get_pti_param(pti_param)
        if pti_param_val == pti_val:
            return True
        else:
            return False

    def check_pti_param_exist(self, pti_param: str) -> bool: 
        """check if pti param exists return True if so"""
        if pti_param in pti_config.PTI_CONFIG:
            return True 
        else:
            return False

    def get_pti_param(self, pti_param:str):
        """get pti params (str)"""
        if self.check_pti_param_exist(pti_param):
            pti_val = param.param_get(pti_param)
            print("current param ", pti_param, "value is ", pti_val)
            return pti_val
        else:
            print("hell naw", pti_param)
            return None


    def set_loop_gain_param(self, pti_loop_gain:str):
        """sets loop gain parameters"""

        print("setting loop gain", pti_config.LOOP_GAIN)

        param.param_set("FTI_LOOP_GAIN",
            pti_config.LOOP_GAIN[pti_loop_gain])

