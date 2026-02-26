using UnityEngine;

namespace AnkleSim.Runtime.UI
{
    public class OrbitCamera : MonoBehaviour
    {
        [SerializeField] private Transform _target;
        [SerializeField] private float _distance = 200f;
        [SerializeField] private float _rotationSpeed = 5f;
        [SerializeField] private float _zoomSpeed = 20f;
        [SerializeField] private float _minDistance = 50f;
        [SerializeField] private float _maxDistance = 500f;

        private float _yaw = 0f;
        private float _pitch = 30f;

        public void FrameBounds(Bounds bounds)
        {
            if (_target == null)
            {
                var go = new GameObject("OrbitTarget");
                _target = go.transform;
            }
            _target.position = bounds.center;
            _distance = Mathf.Clamp(bounds.size.magnitude * 1.5f, _minDistance, _maxDistance);
            UpdatePosition();
        }

        void LateUpdate()
        {
            if (Input.GetMouseButton(1))
            {
                _yaw += Input.GetAxis("Mouse X") * _rotationSpeed;
                _pitch -= Input.GetAxis("Mouse Y") * _rotationSpeed;
                _pitch = Mathf.Clamp(_pitch, -89f, 89f);
            }

            float scroll = Input.GetAxis("Mouse ScrollWheel");
            if (scroll != 0f)
            {
                _distance -= scroll * _zoomSpeed;
                _distance = Mathf.Clamp(_distance, _minDistance, _maxDistance);
            }

            UpdatePosition();
        }

        private void UpdatePosition()
        {
            if (_target == null) return;

            Quaternion rotation = Quaternion.Euler(_pitch, _yaw, 0f);
            Vector3 offset = rotation * new Vector3(0f, 0f, -_distance);
            transform.position = _target.position + offset;
            transform.LookAt(_target.position);
        }
    }
}
