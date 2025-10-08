import RcJoystick from "rc-joystick";

export function Joystick({
  onChange,
  className = "",
  ...props
}: {
  onChange?: (val: any) => void;
  className?: string;
  [key: string]: any;
}) {
  return (
    <div className={`p-2 ${className}`}>
      <RcJoystick
        baseRadius={60}
        controllerRadius={30}
        onChange={onChange}
        {...props}
      />
    </div>
  );
}
