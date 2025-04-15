// JavaScript 연습 #160

// 배열 처리 함수
function processData_160(arr) {
    const sorted = [...arr].sort((a, b) => a - b);
    const sum = arr.reduce((acc, val) => acc + val, 0);
    const avg = sum / arr.length;
    const max = Math.max(...arr);
    const min = Math.min(...arr);

    return {
        sorted,
        sum,
        avg: avg.toFixed(2),
        max,
        min,
        count: arr.length,
    };
}

// 문자열 유틸리티
function formatString_160(str) {
    return str
        .trim()
        .split(/\s+/)
        .map(word => word.charAt(0).toUpperCase() + word.slice(1).toLowerCase())
        .join(' ');
}

// 테스트
const testData = [20, 80, 80, 20, 60];
console.log("연습 #160");
console.log("입력:", testData);
console.log("결과:", processData_160(testData));
console.log("포맷:", formatString_160("hello world javascript"));
