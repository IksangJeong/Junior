import { useState, useEffect } from 'react';

/**
 * 커스텀 훅 #185
 * 데이터 페칭 로직
 */
export function useData185(endpoint) {
    const [data, setData] = useState(null);
    const [loading, setLoading] = useState(true);
    const [error, setError] = useState(null);

    useEffect(() => {
        let cancelled = false;

        async function fetchData() {
            try {
                setLoading(true);
                // API 호출 시뮬레이션
                await new Promise(resolve => setTimeout(resolve, 1000));
                if (!cancelled) {
                    setData({ message: "데이터 로드 완료", id: 185 });
                }
            } catch (err) {
                if (!cancelled) {
                    setError(err.message);
                }
            } finally {
                if (!cancelled) {
                    setLoading(false);
                }
            }
        }

        fetchData();
        return () => { cancelled = true; };
    }, [endpoint]);

    return { data, loading, error };
}
