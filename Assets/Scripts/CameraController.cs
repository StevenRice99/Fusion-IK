using UnityEngine;
using UnityEngine.InputSystem;

/// <summary>
/// Keyboard and mouse camera controls.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(Camera))]
public class CameraController : MonoBehaviour
{
    [Tooltip("Camera movement speed in meters per second.")]
    [Min(float.Epsilon)]
    [SerializeField]
    private float movementSpeed = 1;
    
    [Tooltip("Camera look speed.")]
    [Min(float.Epsilon)]
    [SerializeField]
    private float lookSpeed =  0.1f;
    
    [Tooltip("Camera look speed with the keyboard in meters per second.")]
    [Min(float.Epsilon)]
    [SerializeField]
    private float lookSpeedKeyboard =  30;

    private void LateUpdate()
    {
        Transform t = transform;
        Vector3 position = t.position;
        
        // Upw movement.
        if (Keyboard.current.qKey.isPressed || Keyboard.current.pageUpKey.isPressed)
        {
            position += Vector3.up * (movementSpeed * Time.deltaTime);
        }

        // Down movement.
        if (Keyboard.current.eKey.isPressed || Keyboard.current.pageDownKey.isPressed)
        {
            position += -Vector3.up * (movementSpeed * Time.deltaTime);
        }

        // Left movement.
        if (Keyboard.current.aKey.isPressed || Keyboard.current.leftArrowKey.isPressed)
        {
            position += -t.right * (movementSpeed * Time.deltaTime);
        }

        // Right movement.
        if (Keyboard.current.dKey.isPressed || Keyboard.current.rightArrowKey.isPressed)
        {
            position += t.right * (movementSpeed * Time.deltaTime);
        }

        // Forward movement.
        if (Keyboard.current.wKey.isPressed || Keyboard.current.upArrowKey.isPressed)
        {
            position += t.forward * (movementSpeed * Time.deltaTime);
        }

        // Backwards movement.
        if (Keyboard.current.sKey.isPressed || Keyboard.current.downArrowKey.isPressed)
        {
            position += -t.forward * (movementSpeed * Time.deltaTime);
        }
        
        Vector3 localEulerAngles;
        float newRotationX;
        float newRotationY;

        // Mouse looking.
        if (Mouse.current.rightButton.isPressed)
        {
            localEulerAngles = t.localEulerAngles;
            newRotationX = localEulerAngles.y + Mouse.current.delta.x.ReadValue() * lookSpeed;
            newRotationY = localEulerAngles.x - Mouse.current.delta.y.ReadValue() * lookSpeed;
            t.localEulerAngles = new(newRotationY, newRotationX, 0f);
        }

        t.position = position;
        
        localEulerAngles = t.localEulerAngles;
        newRotationX = localEulerAngles.y;
        newRotationY = localEulerAngles.x;
        
        // Keyboard look left.
        if (Keyboard.current.leftArrowKey.isPressed)
        {
            newRotationX -= lookSpeedKeyboard * Time.deltaTime;
        }

        // Keyboard look right.
        if (Keyboard.current.rightArrowKey.isPressed)
        {
            newRotationX += lookSpeedKeyboard * Time.deltaTime;
        }

        // Keyboard look up.
        if (Keyboard.current.upArrowKey.isPressed)
        {
            newRotationY -= lookSpeedKeyboard * Time.deltaTime;
        }

        // Keyboard look down.
        if (Keyboard.current.downArrowKey.isPressed)
        {
            newRotationY += lookSpeedKeyboard * Time.deltaTime;
        }
            
        t.localEulerAngles = new(newRotationY, newRotationX, 0f);
    }
}