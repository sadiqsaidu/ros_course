import { LucideIcon } from 'lucide-react';

interface LectureSlideProps {
  title: string;
  children: React.ReactNode;
  icon?: LucideIcon;
  id: string;
  subtitle?: string;
}

export default function LectureSlide({ title, children, icon: Icon, id, subtitle }: LectureSlideProps) {
  return (
    <section id={id} className="min-h-screen flex flex-col justify-center py-20 border-b border-zinc-200 last:border-0 bg-white">
      <div className="max-w-5xl mx-auto w-full px-6 md:px-12">
        <div className="flex items-center space-x-3 mb-6 text-zinc-400">
          {Icon && <Icon size={20} />}
          <span className="uppercase tracking-widest text-xs font-bold font-mono">Slide {id.replace('slide-', '')}</span>
        </div>
        <h2 className="text-4xl md:text-5xl font-extrabold text-zinc-900 mb-4 tracking-tight leading-tight">{title}</h2>
        {subtitle && <h3 className="text-xl text-zinc-500 font-light mb-8">{subtitle}</h3>}
        <div className="prose prose-zinc prose-lg max-w-none text-zinc-600 font-light">
          {children}
        </div>
      </div>
    </section>
  );
}
