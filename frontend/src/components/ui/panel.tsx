import { ReactNode } from "react";

type PanelProps = {
  children: ReactNode;
  title?: string;
  variant?: "light" | "dark" | "transparent";
  className?: string;
};

export function Panel({
  title,
  children,
  variant = "light",
  className = "",
}: PanelProps) {
  const base =
    variant === "dark"
      ? "panel-dark"
      : variant === "transparent"
        ? "transparent-panel"
        : "panel";

  return (
    <div className={`${base} p-4 [--zoom:10] ${className}`}>
      {title && <h3 className="text-shadow mb-2 text-lg font-bold">{title}</h3>}
      {children}
    </div>
  );
}
