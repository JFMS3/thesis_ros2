level_q = None


"""
Motive;'s idea of drone frame isn't real drone frame, causing
non-zero rest position.

In publish drone:

q_now = self.transformer.normalise_quat(rotation)

if self.level_q is None:
    self.level_q = q_now.copy()

q_relative = self.transformer.quat_multiply(
    q_now,
    self.transformer.quat_conjugate(self.level_q)
)

q = Quaternion()
q.x, q.y, q.z, q.w = q_relative

phi, theta, psi = self.transformer.quat_to_euler(q)
"""