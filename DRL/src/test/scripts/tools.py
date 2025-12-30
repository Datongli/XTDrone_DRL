import torch
import os
from navigation.scripts.SAC import SAC
from navigation.scripts.DDPG import DDPG
from navigation.scripts.MTransSAC import MTransSAC


def safe_load_state(targetObject: torch.nn.Module | None, stateDict: dict | None) -> None:
    """
    安全加载模型的状态字典（state_dict）
    若模型未定义加载状态字典方法或加载过程中发生异常，忽略，避免崩溃
    :param targetObject: 模型对象
    :param stateDict: 状态字典
    :return: None
    """
    if targetObject is not None and hasattr(targetObject, "load_state_dict") and stateDict is not None:
        try:
            targetObject.load_state_dict(stateDict)
        except Exception:
            pass


def load_pth_models(checkpointDirctory: str, algorithm: DDPG | SAC, device) -> bool:
    """
    加载 .pth 格式的单独模型文件（DDPG 专用）
    :param checkpointDirctory: checkpoint 目录路径
    :param algorithm: 算法实例（DDPG 或 SAC）
    :param device: 设备
    :return: 是否加载成功
    """
    try:
        # 查找 .pth 文件
        actorPath = os.path.join(checkpointDirctory, "actor.pth")
        targetActorPath = os.path.join(checkpointDirctory, "target_actor.pth")
        criticPath = os.path.join(checkpointDirctory, "critic.pth")
        targetCriticPath = os.path.join(checkpointDirctory, "target_critic.pth")
        pthLoad = {
            "actor": actorPath,
            "targetActor": targetActorPath,
            "critic": criticPath,
            "targetCritic": targetCriticPath
        }
        # 检查文件是否存在
        requiredFiles = [actorPath, targetActorPath, criticPath, targetCriticPath]
        missingFiles = [f for f in requiredFiles if not os.path.exists(f)]
        if missingFiles:
            print(f"[测试] 缺少模型文件: {missingFiles}")
            return False
        # 加载模型参数
        print(f"[测试] 正在加载 DDPG 模型文件...")
        for name, pth in pthLoad.items():
            if os.path.exists(pth):
                chechPoint = torch.load(pthLoad[name], map_location=device)
                algorithm.netDict[name].load_state_dict(chechPoint["model"])
                print(f"[测试] ✓ 已加载 {name}: {pth}")
        return True
    except Exception as e:
        print(f"[测试] 加载 .pth 模型失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    

def load_pt_models(checkPointPath: str, navigationAlgorithm: MTransSAC) -> bool:
    """
    加载.pt 模型文件
    :param checkPointPath: checkpoint 路径
    :param navigationAlgorithm: 导航算法实例
    :return: 是否加载成功
    """
    """加载检查点"""
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # 判断是否有可用的设备
    checkPointState: dict[str, any] = torch.load(checkPointPath, map_location=device, weights_only=False)  # 加载检查点状态
    navigationState: dict[str, any] = checkPointState.get("navigartionAlgorithm", {})  # 获取导航算法状态
    """加载导航算法状态（支持注册表自动化加载）"""
    try:
        modulesToLoad = getattr(navigationAlgorithm, "checkPointModules", ["actor", "critic", "targetCritic", "actorOptimizer", "criticOptimizer",
                                  "actorScheduler", "criticScheduler", "logAlpha", "alphaOptimizer",
                                  "difficultyLevel", "updateInterval"])  # 获取需要加载检查点的模块列表
        for moduleName in modulesToLoad:
            # 从算法对象中获取对应的属性
            if not hasattr(navigationAlgorithm, moduleName):
                print(f"导航算法对象中不存在属性 {moduleName}，跳过加载")
                continue  # 若属性不存在，跳过
            targetAttr = getattr(navigationAlgorithm, moduleName)  # 获取目标属性
            savedValue = navigationState.get(moduleName, None)  # 获取保存的属性值
            if savedValue is None:
                print(f"导航算法状态中不存在属性 {moduleName}，跳过加载")
                continue  # 若属性不存在，跳过
            # 如果是模型对象或者优化器，加载参数
            if hasattr(targetAttr, "load_state_dict"):
                safe_load_state(targetAttr, savedValue)
            # 如果是普通属性，加载属性值
            else:
                setattr(navigationAlgorithm, moduleName, savedValue)
        return True
    except Exception as e:
        print(f"加载检查点时获取导航算法状态失败：{e}")
        return False
    

def switch_model_eval_MTransSAC(navigationAlgorithm: any) -> None:
    """
    将MTransSAC导航算法切换为eval模式
    :param navigationAlgorithm: 导航算法对象
    :return: None
    """
    if not hasattr(navigationAlgorithm, "checkPointModules"):
        return
    try:
        modelsToSwitch = getattr(navigationAlgorithm, "checkPointModules", ["actor", "critic", "targetCritic"])
        for model in modelsToSwitch:
            # 只监控有定义且是可调用对象的模块
            if hasattr(model, "eval") and callable(getattr(model, "eval")):
                model.eval()
            else: pass
    except Exception as e:
        print(f"[ERROR] 模型切换eval模式失败：{e}")


def switch_model_eval_DDPG(navigationAlgorithm: any) -> None:
    """
    将DDPG导航算法切换为eval模式
    :param navigationAlgorithm: 导航算法对象
    :return: None
    """
    # actor模型
    actorModule = getattr(navigationAlgorithm, "actor", None)
    if actorModule is not None and hasattr(actorModule, "eval"):  actorModule.eval()
    # targetActor模型
    targetActorModule = getattr(navigationAlgorithm, "targetActor", None)
    if targetActorModule is not None and hasattr(targetActorModule, "eval"):  targetActorModule.eval()
    # critic模型
    critic = getattr(navigationAlgorithm, "critic", None)
    critic1 = getattr(navigationAlgorithm, "critic1", None)
    critic2 = getattr(navigationAlgorithm, "critic2", None)
    if critic is not None and hasattr(critic, "eval"):  critic.eval()
    if critic1 is not None and hasattr(critic1, "eval"):  critic1.eval()
    if critic2 is not None and hasattr(critic2, "eval"):  critic2.eval()
    # targetCritic模型
    targetCritic = getattr(navigationAlgorithm, "targetCritic", None)
    if targetCritic is not None and hasattr(targetCritic, "eval"):  targetCritic.eval()


if __name__ == "__main__":
    pass
