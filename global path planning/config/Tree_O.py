import random


class Tree:
    def __init__(self, node) -> None:
        self.root = node
        self.high = 1
        self.node = []

    def addNode(self, node1, node2) -> None:
        if self.compareNode(node1, node2) != 1:
            left = node1.left if node1.left[0] < node2.left[0] else node2.left
            right = node1.right if node1.right[0] > node2.right[0] else node2.right
            bottom = node1.bottom if node1.bottom[1] < node2.bottom[1] else node2.bottom
            top = node1.top if node1.top[1] > node2.top[1] else node2.top

            newNode = TreeNode([left, right, bottom, top], node2.parent, [node1, node2])
            self.high += 1
            if node2.parent is not None:
                node2.parent.children.remove(node2)
            node2.parent = newNode
            node1.parent = newNode

            if newNode.parent == None:
                self.root = newNode
            return
        else:
            for node in node1.children:
                self.addNode(node, node2)
            node1.children.append(node2)
            node2.parent = node1

    # 随机获取叶节点
    def getNode(self, node, h, g):
        if len(node.children) == 0 or h == g:
            self.node = [node.bottom, node.top]
            return
        x = random.randint(0, len(node.children) - 1)
        self.getNode(node.children[x], h, g + 1)

    # 遍历树
    def viewTree(self, node, c):
        print(c, [node.left, node.right, node.bottom, node.top])
        if len(node.children) == 0:
            return
        for x in node.children:
            self.viewTree(x, c + 1)

    # 计算区域是否重叠
    def compareNode(self, node1, node2):

        # -1没有重叠
        if node1.right[0] <= node2.left[0] or node1.left[0] >= node2.right[0] or node1.top[1] <= node2.bottom[1] or node1.bottom[1] >= node2.top[1]:
            return -1

        # 1被包含
        if node1.left[0] < node2.left[0] and node1.right[0] > node2.right[0] and node1.top[1] > node2.top[1] and node1.bottom[1] < node2.bottom[1]:
            return 1

        # 0为部分重叠
        return 0


class TreeNode:
    def __init__(self, val, parent=None, children=[]):
        self.children = children
        self.parent = parent
        self.left = val[0]
        self.right = val[1]
        self.bottom = val[2]
        self.top = val[3]
