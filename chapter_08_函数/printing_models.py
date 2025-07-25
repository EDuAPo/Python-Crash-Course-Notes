def print_models(unprinted_designs, completed_models):
    """
    模拟打印每个设计，直到没有剩余的设计。
    打印后将每个设计移动到已完成模型中。
    
    参数:
    unprinted_designs: 待打印的设计列表
    completed_models: 已完成打印的设计列表
    """
    # 当待打印设计列表不为空时，继续打印
    while unprinted_designs:
        # 从待打印列表中取出最后一个设计（使用pop()方法）
        current_design = unprinted_designs.pop()
        # 打印当前正在打印的设计
        print(f"Printing model: {current_design}")
        # 将已打印的设计添加到已完成列表中
        completed_models.append(current_design)
        
def show_completed_models(completed_models):
    """
    显示所有已打印的模型。
    
    参数:
    completed_models: 已完成打印的设计列表
    """
    # 打印标题
    print("\nThe following models have been printed:")
    # 遍历所有已完成的设计并打印
    for completed_model in completed_models:
        print(completed_model)
        
# 创建待打印的设计列表
unprinted_designs = ['phone case', 'robot pendant', 'dodecahedron']
# 创建空的已完成设计列表
completed_models = []

# 调用打印函数，处理所有待打印的设计
print_models(unprinted_designs, completed_models)
# 显示所有已完成打印的设计
show_completed_models(completed_models)
