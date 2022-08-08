
import mavros
from afrl_configs import pti_config
from mavros import param

class PTIParamVerifier():
    """Class FTIParamVerifier makes sure the parameters submitted are within the limits
    of the system"""

    def set_pti_param(self, pti_param: str , pti_param_val):
        """checks if pti param exists, not duplicate,
         values are good, and then sets value"""

        if self.check_pti_param_exist(pti_param) == False:
            print("doesn't exist", pti_param)
            return 

        if self.check_dup_param_val(pti_param, pti_param_val) == True:
            print("duplicate value")
            return 

        pti_param_range = pti_config.PTI_CONFIG[pti_param]
 
        #set amplitudes 
        if pti_param == "FTI_FS_AMP_BEGIN" or pti_param == "FTI_FS_AMP_END":
            if pti_param_val in pti_config.input_control_index:
                setting_vals = pti_config.FTI_INJXN_POINT[pti_param_val]
                if setting_vals == None:
                    return 
                
                control_index = pti_config.input_control_index[pti_param_val]
                param.param_set(pti_param, setting_vals[control_index])
                return 
            
            else:
                return 

        #set frequency
        if pti_param_val<= pti_param_range[1] and pti_param_val>=pti_param_range[0]:
            param.param_set(pti_param, pti_param_val)
            print("Set PTI", pti_param, "to", pti_param_val)
            return 
        else:
            print("value is out of range", pti_param_val, 
            "range of values are ", pti_param_range)
            return 

    def check_dup_param_val(self, pti_param: str, pti_param_val) -> bool:
        """check if parameter value is dup/same"""
        pti_val = self.get_pti_params(pti_param)
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

    def get_pti_params(self, pti_param:str):
        """get pti params (str)"""
        if self.check_pti_param_exist(pti_param):
            pti_val = param.param_get(pti_param)
            # print("param value is of ", pti_param, " is ", pti_val)
            return pti_val
        else:
            print("hell naw", pti_param)
            return None


if __name__=='__main__':

    mavros.set_namespace()

    pti = PTIParamVerifier()
    
    val = param.param_set("FTI_INJXN_POINT",2)

    print("val is", val)