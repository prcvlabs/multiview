
#include <algorithm>
#include <iterator>

#define CATCH_CONFIG_PREFIX_ALL
#include "perceive/contrib/catch.hpp"
#include "perceive/cost-functions/localization/localization-data.hpp"

namespace perceive
{
static const string p3ds_dat_str_ = R"V0G0N(
[        
        {
           "best_i":     23,
           "gaze_theta": 2.235371,
           "is_valid":   true,
           "poses":     [{
                 "sensor_no": 0,
                 "detect_no": 0,
                 "pose":       
                    {
                       "model":      "BODY_25",
                       "theta":      2.235371,
                       "score":      0.733801,    
                       "keypoints": [[0, 507.008, 300.308, 0.927176],
                            [1, 503.206, 318.619, 0.901854],
                            [2, 475.917, 316.811, 0.881791],
                            [3, 463.062, 349.729, 0.786861],
                            [4, 453.954, 366.175, 0.67044],
                            [5, 523.436, 320.445, 0.82839],
                            [6, 534.294, 358.922, 0.882776],
                            [7, 534.442, 397.249, 0.808801],
                            [8, 494.186, 389.954, 0.798105],
                            [9, 479.495, 388.132, 0.797142],
                            [10, 474.061, 439.266, 0.801416],
                            [11, 484.981, 439.332, 0.560461],
                            [12, 510.628, 393.574, 0.77384],
                            [13, 503.269, 441.143, 0.848267],
                            [14, 488.735, 483.204, 0.85978],
                            [15, 503.33, 296.708, 0.948527],
                            [16, 514.23, 296.686, 0.879434],
                            [17, 490.432, 298.469, 0.942158],
                            [18, 517.988, 298.445, 0.0699679],
                            [19, 490.557, 503.297, 0.78724],
                            [20, 501.48, 501.488, 0.801555],
                            [21, 486.779, 486.853, 0.681913],
                            [22, 475.883, 448.43, 0.318736],
                            [23, 474.022, 446.683, 0.372291],
                            [24, 486.807, 435.632, 0.416109]],
                       "patches":   [{ "width": 7, "height": 7, "stride": 7, "data": "EkdzAAxBbQAANmEAACZSAAAWPwAAGEEAAClRAB9SfgAXSnYACjtlAAAhSQAAFz0AABM3AAAbPQBJfacAQnOdACRUfAAALFIAABc9AAAWOQAAFjcAaJvCAG2dxQBQfKIAFDxhAAAWOgAAEjQAABQ0AIOz2wCCsNgAWIGoABg7YQAAGjwAABY1AAAWNwCMvOQAfKfQAFmApwAlSG4AABs9AAAWNwACGDYAhrTaAGuXvQBIcJcAHD5hAAEdPAABGTcABho5AA==" },
                            { "width": 7, "height": 7, "stride": 7, "data": "Ax0oAAQbJwADGSgAAhUnAAIVJwAAFicAABYnAAQeKAACGycAARomAAAXJQABFCYAABYnAAAVJgAEHigABBsnAAMaJgAAFyUAABUmAAAVJgAAFSYABB4oAAQbJwADGiYAABclAAAVJgABFCYAARQmAAYdKQAEGycAAxkoAAAWJwACFScAAhUnAAATJAAGHSkABBsnAAMZKAAAFicAAhUnAAIVJwABFCYABh0pAAQaKQADGSgAAhUnAAIVJwACFScAARQmAA==" },
                            { "width": 7, "height": 7, "stride": 7, "data": "AxAnAAIMJgAACyAAAAwfAAAMHwAADBwAAAwcAAANJAACDSQAAAsgAAAMHwAADB8AAAwcAAAMHAAADSEAAAsgAAALHgAADBwAAAwcAAANGgAADRoAAA0dAAALHgAADBwAAAwcAAAMHAAADRoAAA0aAAAMHAAACx4AAAwcAAANGgAADRoAAA0aAAANGgAADBwAAA0aAAANGgAADRoAAA0aAAANGgAADRoAAA0aAAANGgAADRoAAA0aAAANGgABDRkAAQ0ZAA==" }]
                    }
              
              }],
           "Cs":        [[1.764468, -1.287303], [1.725583, -1.232254], [1.686697, -1.177205], [1.647812, -1.122156], [1.608927, -1.067107], [1.570041, -1.012057], [1.531156, -0.957008], [1.492270, -0.901959], [1.453385, -0.846910], [1.414499, -0.791861], [1.375614, -0.736812], [1.336728, -0.681762], [1.297843, -0.626713], [1.258957, -0.571664], [1.220072, -0.516615], [1.181187, -0.461566], [1.142301, -0.406516], [1.103416, -0.351467], [1.064530, -0.296418], [1.025645, -0.241369], [0.986759, -0.186320], [0.947874, -0.131271], [0.908988, -0.076221], [0.870103, -0.021172], [0.831217, 0.033877], [0.792332, 0.088926], [0.753447, 0.143975], [0.714561, 0.199024], [0.675676, 0.254074], [0.636790, 0.309123], [0.597905, 0.364172], [0.559019, 0.419221], [0.520134, 0.474270], [0.481248, 0.529320], [0.442363, 0.584369], [0.403477, 0.639418], [0.364592, 0.694467], [0.325707, 0.749516], [0.286821, 0.804565], [0.247936, 0.859615]],
           "ius":       [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000002, 0.000005, 0.000012, 0.000023, 0.000041, 0.000063, 0.000085, 0.000099, 0.000099, 0.033973, 0.025668, 0.208553, 0.404681, 0.251766, 0.074929, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
        }
        ,
        
        {
           "best_i":     22,
           "gaze_theta": 2.508073,
           "is_valid":   true,
           "poses":     [{
                 "sensor_no": 0,
                 "detect_no": 1,
                 "pose":       
                    {
                       "model":      "BODY_25",
                       "theta":      2.508073,
                       "score":      0.444379,    
                       "keypoints": [[0, 755.642, 388.069, 0.707905],
                            [1, 706.24, 410.046, 0.716254],
                            [2, 682.482, 408.227, 0.723156],
                            [3, 746.469, 479.509, 0.654914],
                            [4, 761.091, 413.7, 0.723398],
                            [5, 728.183, 413.661, 0.577075],
                            [6, 757.468, 474.015, 0.236215],
                            [7, 0, 0, 0],
                            [8, 682.464, 527.055, 0.566641],
                            [9, 684.304, 530.708, 0.513082],
                            [10, 698.942, 587.385, 0.383615],
                            [11, 711.688, 667.849, 0.185597],
                            [12, 687.942, 527.049, 0.364714],
                            [13, 658.678, 547.188, 0.382473],
                            [14, 622.14, 592.932, 0.764022],
                            [15, 751.923, 375.332, 0.787962],
                            [16, 761.092, 375.362, 0.0979861],
                            [17, 720.843, 371.582, 0.755255],
                            [18, 0, 0, 0],
                            [19, 653.199, 622.2, 0.551858],
                            [20, 651.338, 620.277, 0.512521],
                            [21, 609.366, 596.51, 0.659263],
                            [22, 708.079, 667.887, 0.0502957],
                            [23, 708.066, 667.893, 0.0625942],
                            [24, 713.576, 667.868, 0.132674]],
                       "patches":   [{ "width": 7, "height": 7, "stride": 7, "data": "IlAAACBPAAAfTgAAKFYAADltAgBCegQATYYLACpiAAAuYQAAJlcAACVVAAAtYAAAPHMBAEqECQBDgwAAPncAADRmAQAnWQAAIlUAACtgAAA1bAAAdLQfAE+OAgBCewIAM2YAACdbAAAlWQAALGIAAI/QNgBtrRgAV5YMAEiCBgA6cAAALmEAACtfAACa3j8AiMszAHO0IwBinhoAU4sSAEJ4BwA1awAAn+BAAJXWPAB+wCoAbq4iAGKgFgBTjwoAQX0AAA==" },
                            { "width": 7, "height": 7, "stride": 7, "data": "MWMQADJjFAA2aBkAO2obAD9sHgBHdCQAUX0rAC9iDwAyZREAM2UUADZoGQA7ahsAPm4dAEd1IwAxYxAAMmURADJlEQA0ZhUAOGgXADlsGAA+bxwAMWMQADFjEAAxYxAAMmURADRmFQA1ZxYAOWsaADFjEgAxYxAAMWMSADJlEQAzZRQANGYVADZoFwAzZRQAM2UUADNlFAAyZBMAMmQTADRmFQA1ZxYAM2UUADNlFAAzZRQAM2UUADRmFQA0ZhUANGYVAA==" },
                            { "width": 7, "height": 7, "stride": 7, "data": "f7MrAH21KgCBuDAAhLsyAIe+NgCFvzYAhb43AIG1LwB/ti4AgbgwAIK6MQCFuzUAhr01AIa8NgCDtjMAg7cyAIO3MgCFuDMAhro0AIW7NQCFuzUAhbg0AIO2MwCFuDMAhbgzAIa6NACFuzUAhbs1AIa4NwCFtzYAhbg0AIa5NgCFuDQAhrk2AIe6NwCGtzkAhrg3AIW3NgCGuTYAhbc2AIa5NgCGuTYAgbQ4AIK0NQCCtDUAgrU0AIK1NACDtjMAhbg0AA==" }]
                    }
              
              }],
           "Cs":        [[-0.297061, -0.965100], [-0.303006, -0.915200], [-0.308951, -0.865299], [-0.314896, -0.815398], [-0.320841, -0.765497], [-0.326786, -0.715596], [-0.332731, -0.665695], [-0.338676, -0.615795], [-0.344621, -0.565894], [-0.350566, -0.515993], [-0.356511, -0.466092], [-0.362456, -0.416191], [-0.368401, -0.366291], [-0.374347, -0.316390], [-0.380292, -0.266489], [-0.386237, -0.216588], [-0.392182, -0.166687], [-0.398127, -0.116786], [-0.404072, -0.066886], [-0.410017, -0.016985], [-0.415962, 0.032916], [-0.421907, 0.082817], [-0.427852, 0.132718], [-0.433797, 0.182618], [-0.439742, 0.232519], [-0.445687, 0.282420], [-0.451632, 0.332321], [-0.457577, 0.382222], [-0.463522, 0.432122], [-0.469467, 0.482023], [-0.475412, 0.531924], [-0.481357, 0.581825], [-0.487302, 0.631726], [-0.493247, 0.681626], [-0.499192, 0.731527], [-0.505137, 0.781428], [-0.511082, 0.831329], [-0.517027, 0.881230], [-0.522972, 0.931131], [-0.528917, 0.981031]],
           "ius":       [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000994, 0.003250, 0.007121, 0.014587, 0.026478, 0.043334, 0.064475, 0.086998, 0.107341, 0.122037, 0.126330, 0.118359, 0.099024, 0.074178, 0.049571, 0.029086, 0.015176, 0.007127, 0.002991, 0.001103, 0.000345, 0.000083, 0.000011, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
        }
        ,
        
        {
           "best_i":     21,
           "gaze_theta": -3.048758,
           "is_valid":   true,
           "poses":     [{
                 "sensor_no": 2,
                 "detect_no": 0,
                 "pose":       
                    {
                       "model":      "BODY_25",
                       "theta":      -3.048758,
                       "score":      0.624910,    
                       "keypoints": [[0, 4.13105, 110.161, 0.775798],
                            [1, 22.4348, 148.599, 0.830245],
                            [2, 5.03549, 166.827, 0.55497],
                            [3, 18.7167, 214.4, 0.588084],
                            [4, 46.1916, 241.879, 0.448574],
                            [5, 49.8867, 126.642, 0.825716],
                            [6, 95.5046, 166.952, 0.844935],
                            [7, 130.283, 208.882, 0.744839],
                            [8, 80.9829, 230.913, 0.738085],
                            [9, 64.4759, 241.891, 0.752431],
                            [10, 95.5019, 300.354, 0.805996],
                            [11, 122.989, 338.696, 0.517467],
                            [12, 102.867, 223.533, 0.700455],
                            [13, 128.476, 283.917, 0.821999],
                            [14, 148.569, 329.596, 0.833003],
                            [15, 5.03504, 110.133, 0.583942],
                            [16, 4.12813, 102.905, 0.579304],
                            [17, 5.05081, 121.2, 0.226781],
                            [18, 18.7511, 104.684, 0.116413],
                            [19, 154.016, 342.406, 0.784359],
                            [20, 161.374, 338.755, 0.72288],
                            [21, 144.955, 333.283, 0.638925],
                            [22, 132.134, 344.196, 0.408411],
                            [23, 122.976, 344.269, 0.388323],
                            [24, 124.889, 336.893, 0.39082]],
                       "patches":   [{ "width": 7, "height": 7, "stride": 7, "data": "BklnAAA4UQAAIkAAAB47AAIbNwACGTMAAhkzABBVcwABRl8AAC1JAAAiPgAAGjYAABozAAAaMQAVW3kACU5pAAQ0UAAAJEEAABo2AAEbNAAAGjEAG2CAABJWcQAIPVsAACZCAAAaNQAAGzEAAB0xACRtjwASV3cACUNjAAAqRAAAGDIAABw0AAAbMQAwfp8AHmiHABJTcgABO1MAACY+AAAeMwAAHTAANIapADF/oAAfZ4QAElNrAAU7UgAAKj0AAB8xAA==" },
                            { "width": 7, "height": 7, "stride": 7, "data": "AREZAAERGQAAEBsAABEcAAASHAABFB8AARYiAAEQGwADDxsAARAbAAIRHAAAEhsAARQfAAEWIgACERwAAhEcAAISGwAEExwAAhMcAAEUHwABFyEAAxQfAAISHQACExwABBMcAAITHAABFB8AARchAAIVIAADFB8AAxQfAAUTHwACEh0AAxQfAAMWIQACFSAAARQfAAMUHwAFEx8AAhIdAAMUHwACFSAAARchAAIVIAABFB8AAxQfAAISHQADFB8AAhUgAA==" },
                            { "width": 7, "height": 7, "stride": 7, "data": "AAobAAAJHAAACRwAAAkeAAAJHgAACR4AAAofAAAKHAAACRwAAAkcAAAJHgAACR4AAAkeAAAKHwAAChwAAAkcAAAJHAAACR4AAAkeAAAJHgAACh8AAAseAAAJHgAACR4AAAkeAAAJHgAACR4AAAofAAAKHwABCh8AAQofAAAJHgAACCAAAAkeAAAKHwAACyAAAQofAAEKHwAACCAAAAggAAAJHgAACh8AAAsgAAEKHwABCh8AAAggAAAIIAAACR4AAAofAA==" }]
                    }
              
              }],
           "Cs":        [[1.481825, -1.007157], [1.451535, -0.956393], [1.421244, -0.905628], [1.390953, -0.854864], [1.360662, -0.804099], [1.330372, -0.753334], [1.300081, -0.702570], [1.269790, -0.651805], [1.239500, -0.601040], [1.209209, -0.550276], [1.178918, -0.499511], [1.148628, -0.448747], [1.118337, -0.397982], [1.088046, -0.347217], [1.057755, -0.296453], [1.027465, -0.245688], [0.997174, -0.194923], [0.966883, -0.144159], [0.936593, -0.093394], [0.906302, -0.042630], [0.876011, 0.008135], [0.845720, 0.058900], [0.815430, 0.109664], [0.785139, 0.160429], [0.754848, 0.211194], [0.724558, 0.261958], [0.694267, 0.312723], [0.663976, 0.363488], [0.633685, 0.414252], [0.603395, 0.465017], [0.573104, 0.515781], [0.542813, 0.566546], [0.512523, 0.617311], [0.482232, 0.668075], [0.451941, 0.718840], [0.421650, 0.769605], [0.391360, 0.820369], [0.361069, 0.871134], [0.330778, 0.921898], [0.300488, 0.972663]],
           "ius":       [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000001, 0.000001, 0.000002, 0.000003, 0.000003, 0.001421, 0.001798, 0.031542, 0.044144, 0.271963, 0.303170, 0.224005, 0.108599, 0.008928, 0.004418, 0.000001, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000]
        }
        ,
        
        {
           "best_i":     20,
           "gaze_theta": 2.154506,
           "is_valid":   true,
           "poses":     [{
                 "sensor_no": 2,
                 "detect_no": 1,
                 "pose":       
                    {
                       "model":      "BODY_25",
                       "theta":      2.154506,
                       "score":      0.587435,    
                       "keypoints": [[0, 342.341, 93.6987, 0.54809],
                            [1, 314.963, 133.937, 0.770622],
                            [2, 300.36, 139.529, 0.808326],
                            [3, 371.608, 181.421, 0.787173],
                            [4, 366.178, 119.335, 0.694102],
                            [5, 329.609, 124.777, 0.668222],
                            [6, 371.664, 163.225, 0.213629],
                            [7, 366.164, 119.32, 0.156992],
                            [8, 342.397, 232.704, 0.717073],
                            [9, 335.113, 238.149, 0.692501],
                            [10, 377.183, 280.247, 0.762142],
                            [11, 399.035, 329.554, 0.798573],
                            [12, 347.898, 229.039, 0.625992],
                            [13, 336.882, 280.232, 0.620216],
                            [14, 331.404, 329.596, 0.837464],
                            [15, 331.402, 84.5739, 0.520222],
                            [16, 342.395, 81.0172, 0.162577],
                            [17, 302.221, 86.4056, 0.644839],
                            [18, 0, 0, 0],
                            [19, 360.681, 331.487, 0.644543],
                            [20, 356.984, 329.578, 0.596453],
                            [21, 327.719, 336.908, 0.662782],
                            [22, 415.493, 329.524, 0.536363],
                            [23, 411.897, 331.469, 0.591631],
                            [24, 395.398, 336.913, 0.625351]],
                       "patches":   [{ "width": 7, "height": 7, "stride": 7, "data": "DVEPAA1PEgAQUhMAEFgRABNbFAATXREAEl0NAA1LCwANSBEADUwSAA5SEAARVREAE1gSABFaDwA+hSoALXYYACNmFwAcYBcAGV4YABJZDgAJUAMAcsVPAHXHUwBdrEIATJY6AEKKNgA6gywANH4lAKX9egCP52QAfdJVAHLGTgBisUAAY7RFAF2wPACf92wAnPVnAJjwZQCe9WwAjeNcAJbsZwCO52AAo/1rAKP9awCj/WwAoPpqAKH5aQCc92IAl/JfAA==" },
                            { "width": 7, "height": 7, "stride": 7, "data": "SahAAEqpQQBKqUEATqpCAFGqQwBSrEQAVqtEAEinPQBIpz0ASKc9AEinPQBLqD4ATKk/AFCqQABJpjwASaY8AEmmPABHpjwASaY8AEqnPQBLqD4AS6U8AEmmPABLpTwASaY8AEmmPABHpjwASKQ8AEqkOgBKpDoASqQ6AEilOgBIpToASKQ8AEWjOwBHnzYARaA2AEefNgBDoDYAQ6A2AEOfNwBCnjYAQZkwAD6ZLwBAmC8APpgwAD6YMAA8lzIAO5YxAA==" },
                            { "width": 7, "height": 7, "stride": 7, "data": "X7IuAF+yLgBesS0AXa8uAF2uLwBbry8AWa0uAGCzLwBgsy8AX7IuAF2vLgBdri8AWrAwAFmvLwBisy8AYLMvAF+yLgBbriwAW6wwAFqwMABZrjEAX7AqAFyuKABZrCgAV6wqAFaqKgBVqy0AVqwuAFqsJgBYqSMAVKkiAFOnIwBSpCUATqUlAE6lJQBVqB8AU6UdAE+jHABLnhoASJoZAEKaFwBAlxcAVKYgAE6gGgBMnhgAR5oWAEeZFwA0jAkANIsLAA==" }]
                    }
              
              }],
           "Cs":        [[-0.389550, -0.723258], [-0.389008, -0.677171], [-0.388465, -0.631084], [-0.387922, -0.584997], [-0.387379, -0.538910], [-0.386837, -0.492823], [-0.386294, -0.446736], [-0.385751, -0.400649], [-0.385208, -0.354562], [-0.384666, -0.308475], [-0.384123, -0.262388], [-0.383580, -0.216301], [-0.383038, -0.170214], [-0.382495, -0.124127], [-0.381952, -0.078040], [-0.381409, -0.031953], [-0.380867, 0.014134], [-0.380324, 0.060221], [-0.379781, 0.106308], [-0.379238, 0.152395], [-0.378696, 0.198482], [-0.378153, 0.244569], [-0.377610, 0.290656], [-0.377067, 0.336743], [-0.376525, 0.382830], [-0.375982, 0.428917], [-0.375439, 0.475004], [-0.374896, 0.521091], [-0.374354, 0.567178], [-0.373811, 0.613265], [-0.373268, 0.659352], [-0.372725, 0.705439], [-0.372183, 0.751526], [-0.371640, 0.797613], [-0.371097, 0.843700], [-0.370555, 0.889787], [-0.370012, 0.935874], [-0.369469, 0.981961], [-0.368926, 1.028048], [-0.368384, 1.074135]],
           "ius":       [0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000000, 0.000066, 0.001785, 0.005617, 0.012558, 0.023258, 0.037548, 0.053963, 0.070680, 0.087331, 0.101553, 0.108935, 0.107819, 0.099680, 0.085790, 0.068479, 0.050466, 0.034701, 0.022344, 0.013531, 0.007544, 0.003781, 0.001667, 0.000638, 0.000207, 0.000053, 0.000007, 0.000000, 0.000000, 0.000000, 0.000000]
        }
]
)V0G0N"s;

template<typename T> void fill_container(T& im, int w, int h)
{
   typename T::value_type counter = 0;
   im.resize(w, h);
   for(auto y = 0; y < h; ++y)
      for(auto x = 0; x < w; ++x) im(x, y) = counter++;
}

static LocalizationData make_localization_data()
{
   LocalizationData ldat;
   ldat.loc_min = -20.0;
   ldat.loc_max = 20.0;
   ldat.hist_sz = 0.11;
   ldat.bounds  = AABB(-1.0, -1.0, 10.0, 11.0);

   int w = 256, h = 256;
   fill_container(ldat.loc_hist, w, h);
   // fill_container(ldat.labels, w, h);

   WARN("testcase is omitting p2ds");

   // Json::Value p3ds_json = parse_json(p3ds_dat_str_);
   // CATCH_REQUIRE(p3ds_json.type() == Json::arrayValue);
   // ldat.p3ds.clear();
   // ldat.p3ds.reserve(p3ds_json.size());
   // for(const auto& node : p3ds_json) ldat.p3ds.push_back(Pose3D::read(node));

   // // Okay, are the p3ds good?
   // for(const auto& p3d : ldat.p3ds) {
   //    const auto json_val = p3d.to_json();
   //    const auto o        = Pose3D::read(json_val);
   //    CATCH_REQUIRE(p3d == o);
   // }

   return ldat;
}

CATCH_TEST_CASE("LocalizationData", "[localization-data]")
{
   CATCH_SECTION("localization-data")
   {
      LocalizationData ldat = make_localization_data();
      LocalizationData rhs;

      vector<char> buffer(1024 * 1024); // 1meg
      FILE* fp = nullptr;

      fp = fmemopen(&buffer[0], buffer.size(), "w");
      ldat.write(fp);
      fclose(fp);

      fp = fmemopen(&buffer[0], buffer.size(), "r");
      rhs.read(fp);
      fclose(fp);

      CATCH_REQUIRE(ldat == rhs);
   }
}

} // namespace perceive
