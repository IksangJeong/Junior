"""
데이터베이스 연습 #328
SQLite CRUD 실습
"""
import sqlite3

def setup_328():
    conn = sqlite3.connect(':memory:')
    cursor = conn.cursor()
    cursor.execute('''CREATE TABLE items_328 (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        name TEXT NOT NULL,
        value REAL,
        category TEXT
    )''')

    data = [
        [('item_328_0', 961.85, 'C'), ('item_328_1', 152.75, 'C'), ('item_328_2', 822.69, 'A'), ('item_328_3', 306.28, 'C'), ('item_328_4', 248.07, 'B'), ('item_328_5', 680.26, 'C'), ('item_328_6', 338.41, 'C'), ('item_328_7', 148.55, 'C')]
    ]
    cursor.executemany(
        'INSERT INTO items_328 (name, value, category) VALUES (?, ?, ?)',
        data
    )
    conn.commit()
    return conn

if __name__ == '__main__':
    conn = setup_328()
    cursor = conn.cursor()

    print(f"DB 연습 #328")
    cursor.execute('SELECT * FROM items_328')
    for row in cursor.fetchall():
        print(f"  {row}")

    cursor.execute('SELECT category, COUNT(*), AVG(value) FROM items_328 GROUP BY category')
    print("\n카테고리별 통계:")
    for row in cursor.fetchall():
        print(f"  {row[0]}: {row[1]}개, 평균 {row[2]:.2f}")

    conn.close()
