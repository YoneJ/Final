class PoseManager:
    _instance = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super(PoseManager, cls).__new__(cls, *args, **kwargs)
            cls._instance.pose = None  # Initialize pose as None
        return cls._instance

    def set_pose(self, pose):
        self.pose = pose

    def get_pose(self):
        return self.pose
